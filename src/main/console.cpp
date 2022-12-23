// Elmo Trolla, 2021
// Licence: pick one - public domain / UNLICENCE (https://www.unlicense.org) / MIT (https://opensource.org/licenses/MIT).

// Listens on ESP32 uart, registers "help" and "tasks" console commands, and allows sending/receiving cobs-encoded
// packets in between typing console commands.
//
// Simplified logic for switching between human-interactive and cobs-packetized mode:
//
//     if STATE_MACHINE_PACKETS:
//         if time from last byte > 0.5 s:
//             state = STATE_HUMAN_INTERACTIVE # and handle this byte in the next state, resuming any input
//
//     if STATE_HUMAN_INTERACTIVE:
//           if c == esc:
//               clear current line buf
//           if c == 0:
//               restart machinepackets state
//               state = STATE_MACHINE_PACKETS
//

#include "console.h"

//#include <stdio.h> // sprintf
#include "unistd.h" // fsync

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_system.h"
#include "esp_pm.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"

#include "cobs.h"
#include "sys_globals.h"


static const char* TAG = "console";


#ifndef TASK_PRIORITY_CONSOLE_UART
	#define TASK_PRIORITY_CONSOLE_UART 13
#endif

#if CONFIG_LOG_COLORS
	#define PROMPT LOG_BOLD(LOG_COLOR_RED) "cmd> " LOG_RESET_COLOR
#else
	#define PROMPT "cmd> "
#endif

#define PROTOCOL_MAX_PACKET_BODY_SIZE 128 // TODO: replace

#define L_UART_NUM    CONFIG_ESP_CONSOLE_UART_NUM

// TODO: for >= HW_VERSION_3 the console uart is a non-wakeupable uart. so the cpu has to wake from light sleep
//       periodically (once every 2.5 seconds?) and see if there are any bytes in the rx buf. zeroes?
//       if yes, shall stay awake for 30 seconds.


#if USE_HW_VERSION >= HW_VERSION_3
	#define L_UART_TX_PIN GPIO_NUM_4  // PINS_CPU.nfc_board_uart_tx = 4
	#define L_UART_RX_PIN GPIO_NUM_34 // PINS_CPU.nfc_board_uart_rx = 34
#else
	#define L_UART_TX_PIN GPIO_NUM_1
	#define L_UART_RX_PIN GPIO_NUM_3
#endif


// rx ringbuf size has to be larger than UART_FIFO_LEN (which seems to be 128).
// tx ringbuf size can be 0, or also has to be larger than UART_FIFO_LEN, or uart_driver_install fails.
#define L_ESP_RX_RINGBUF_SIZE 129
#define L_ESP_TX_RINGBUF_SIZE 129

#define L_CONSOLE_CMD_MAX_LEN 129

#define L_MAX_DECODED_PACKET_SIZE PROTOCOL_MAX_PACKET_BODY_SIZE // from protocol_support.h

#define SERIALCOMM_CRC_BYTES 2
#define l_cobs_encode_crc cobs_encode_crc16
#define l_cobs_decode_crc cobs_decode_crc16
#define l_cobs_encode_crc_frame_overhead cobs_encode_crc16_frame_overhead
#define l_cobs_encode_crc_with_frame_overhead cobs_encode_crc16_with_frame_overhead

#define USE_LIGHT_SLEEP

static void          l_console_register_commands();
static void          l_rx_packet(u8* data, u8 len);
static inline void   l_rx_byte(u8 byte);
static int           l_prompt_len;

[[noreturn]] static void l_uart_event_task(void *pvParameters);

static bool          l_initialized;
static QueueHandle_t l_uart_queue;



#ifdef USE_LIGHT_SLEEP

	// A workaround for esp-idf where the cpu is put to sleep in the middle of receiving
	// data from uart. Another piece of the puzzle is in main.cpp:l_idle_task_cb, where
	// the cpu is prevented from going to sleep if uart is still sending data out.

	// Logic:
	//     every time something is received from uart, a power management lock is taken
	// to prevent cpu from sleeping, and a timer is started to later release the lock.
	// If new data comes in while the timer is running, a flag is raised
	// (l_pm_lock_release_timer_restart) so that the timer handler knows it has to restart
	// the timer .

	#define PM_LOCK_RELEASE_TIMEOUT_US 60000000 // 60 seconds (implementation detail: real timeout is dynamic between 60..120 seconds)
	static esp_pm_lock_handle_t l_uart0_rx_pm_lock; // TODO: rename
	static SemaphoreHandle_t    l_pm_lock_timer_mutex;
	static StaticSemaphore_t    l_pm_lock_timer_mutex_buffer;
	static bool                 l_pm_lock_acquired;
	static bool                 l_pm_lock_release_timer_restart;
	static esp_timer_handle_t   l_pm_lock_release_timer_handle;

	static inline void l_pm_lock_poke_timer();
	static void        l_pm_lock_release_timer_callback(void* arg);

#endif



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// public interface
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


u32 g_debug_console_rx_packets_broken; // packet too long, too short, framing error (multiple packets got probably combined), cobs decode error.
//u32 g_debug_rfid_rx_packets_dropped; // no room in rx packet buf
u32 g_debug_console_rx_crc_errors;
//u32 g_debug_rfid_tx_packets_dropped; // no room in tx byte ringbuf


void console_init() {

	l_prompt_len = strlen(PROMPT);

	// Disable buffering on stdin
	setvbuf(stdin, NULL, _IONBF, 0);

	uart_config_t uart_config = {};
	uart_config.baud_rate  = CONFIG_ESP_CONSOLE_UART_BAUDRATE;
	uart_config.data_bits  = UART_DATA_8_BITS;
	uart_config.parity     = UART_PARITY_DISABLE;
	uart_config.stop_bits  = UART_STOP_BITS_1;
	uart_config.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
	uart_config.source_clk = UART_SCLK_REF_TICK;

	esp_err_t err = ESP_OK;
	//err |= uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, L_ESP_RX_RINGBUF_SIZE, 0, 0, NULL, 0);
	err |= uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, L_ESP_RX_RINGBUF_SIZE, L_ESP_TX_RINGBUF_SIZE, 20, &l_uart_queue, 0);
	err |= uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config);

	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to setup console UART. last error: %s", esp_err_to_name(err));
		l_initialized = false;
		return;
	}

	//esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM); // also turns on blocking for getchar()

	// turn off any newline modifications on the output side. input should not matter as we should not use
	// any functions that read from the stdin port (getchar, scanf, ..) because we handle uart input ourselves.
	esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM,ESP_LINE_ENDINGS_LF);

	// Move the caret to the beginning of the next line on '\n'. seems to be the default value anyway.
	// Replaces \n chars on printf and ESP_LOG* and other. Without this, not usable over simple terminals
	// like putty or minicom.
	// TODO: does this replace even in uart_write_bytes() and uart_write_bytes()?
	//       would be fucking bad if it did. no way to use binary protocol then.
	esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);

	// hm. interesting. since we use here event-based logic, nothing will ever be given to stdin.
	// but on what function does the getchar & co wait then? will it conflict with the event-based method?

	//err |= uart_set_pin(L_UART_NUM, L_UART_TX_PIN, L_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	//uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, max_cmdline_length * 2, max_cmdline_length * 2, 20, &uart_0_queue, 0);

	//NB! uart_set_rx_timeout() is not working for uart_read_bytes in default mode! have to use the event-based api

	#ifdef USE_LIGHT_SLEEP

		esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "console_uart", &l_uart0_rx_pm_lock);
		l_pm_lock_timer_mutex = xSemaphoreCreateMutexStatic(&l_pm_lock_timer_mutex_buffer);

		const esp_timer_create_args_t timer_args = {
			.callback = &l_pm_lock_release_timer_callback,
			.arg = nullptr,
			.dispatch_method = ESP_TIMER_TASK,
			.name = "console_uart_pm"
		};
		ESP_ERROR_CHECK(esp_timer_create(&timer_args, &l_pm_lock_release_timer_handle));

	#endif

	// oh, 2048 bytes was too little when log-debugging
	xTaskCreate(l_uart_event_task, "console", 3072, NULL, TASK_PRIORITY_CONSOLE_UART, NULL); // TODO: how much stack do we need?

	esp_console_config_t console_config = {
		.max_cmdline_length = L_CONSOLE_CMD_MAX_LEN,
		.max_cmdline_args = 8,
		#if CONFIG_LOG_COLORS
			.hint_color = atoi(LOG_COLOR_CYAN),
		#else
			.hint_color = 0,
		#endif
		.hint_bold = 1,
	};

	ESP_ERROR_CHECK( esp_console_init(&console_config) );

	esp_console_register_help_command();
	l_console_register_commands();

	/* Move the caret to the beginning of the next line on '\n' */
	//esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);

	l_initialized = true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// private functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#if (configENABLE_TASK_SNAPSHOT != 1)
	#error need configENABLE_TASK_SNAPSHOT for stack size reporting
#endif


// Mirrors the hidden TCB_t (typedef struct tskTaskControlBlock) type in freertos/tasks.c.
// We need this because there's no other way to get task stack size.
// If freertos implementation changes then, well, our stack size reporting will report garbage.
struct l_freertos_TCB_start_t {
	volatile StackType_t* pxTopOfStack;
	#if (portUSING_MPU_WRAPPERS == 1)
		xMPU_SETTINGS xMPUSettings;
	#endif
	ListItem_t   xGenericListItem;
	ListItem_t   xEventListItem;
	UBaseType_t  uxPriority;
	StackType_t* pxStack;
	char         pcTaskName[configMAX_TASK_NAME_LEN];
	BaseType_t   xCoreID;
	#if (portSTACK_GROWTH > 0 || configENABLE_TASK_SNAPSHOT == 1)
		StackType_t *pxEndOfStack;
	#endif
	//
	// a lot of things follow, but we need only access to pxStack and pxEndOfStack
	// ..
	//
};

static void l_vTaskList() {

	u32 num_tasks = uxTaskGetNumberOfTasks();
	TaskStatus_t tasks_status_array[num_tasks];
	u32 total_run_time;
	u32 total_stack = 0;
	const char* status_str;

	num_tasks = uxTaskGetSystemState(tasks_status_array, num_tasks, &total_run_time);

	fputs("-----------------------------------------------------------------\n", stdout);
	fputs("Task_Name       Status    Prio Task# Core stack_B StackHWM Stack%\n", stdout);
	fputs("-----------------------------------------------------------------\n", stdout);

	// Create a human readable table from the binary data in tasks_status_array.
	for (int i = 0; i < num_tasks; i++) {
		switch (tasks_status_array[i].eCurrentState) {
		case eReady:     status_str = "ready"; break;
		case eBlocked:	 status_str = "blocked"; break;
		case eSuspended: status_str = "suspended"; break;
		case eDeleted:	 status_str = "deleted"; break;
		default:         status_str = "?unknown?"; break;
		}

		TaskStatus_t* status = &tasks_status_array[i];

		auto tcb = (l_freertos_TCB_start_t*)status->xHandle;
		i32 cpu_core = tcb->xCoreID == tskNO_AFFINITY ? -1 : tcb->xCoreID; // -1 is any core possible

		u32 task_stack_size_bytes = tcb->pxEndOfStack - tcb->pxStack; // interesting. 4 bytes less than what we wanted in xTaskCreate.
		u32 stack_high_water_mark_bytes = status->usStackHighWaterMark * sizeof(StackType_t); // yes, need the multiplication
		u32 stack_usage_percent = (f32)stack_high_water_mark_bytes * 100.f / (f32)task_stack_size_bytes + 0.5f; // TODO: fixed-point rounding how? without using floats.

		total_stack += task_stack_size_bytes + 4;

		#if configTASKLIST_INCLUDE_COREID
			printf("%-15s %-9s  % 3i   % 3i   % 2i   % 5i    % 5i   % 3i%%\n",
				status->pcTaskName, status_str, status->uxCurrentPriority,
				status->xTaskNumber, cpu_core,
				task_stack_size_bytes, stack_high_water_mark_bytes, stack_usage_percent);
		#else
			printf("%-15s %-9s  % 3i   % 3i    -   % 5i    % 5i   % 3i%%\n",
				status->pcTaskName, status_str, status->uxCurrentPriority,
				status->xTaskNumber,
				task_stack_size_bytes, stack_high_water_mark_bytes, stack_usage_percent);
		#endif
	}
	fputs("-----------------------------------------------------------------\n", stdout);
	printf("total stack capacity    : %i\n", total_stack);
	printf("current free heap size  : %i\n", esp_get_free_heap_size());
	printf("min heap size since boot: %i\n", (i32)heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT));

	// To find the amount of statically allocated memory, use the idf.py size command.
	// Due to a technical limitation, the maximum statically allocated DRAM usage is 160KB.
	// The remaining 160KB (for a total of 320KB of DRAM) can only be allocated at runtime as heap.

	//heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);
	//heap_caps_print_heap_info(MALLOC_CAP_INTERNAL);
	//heap_caps_print_heap_info(MALLOC_CAP_SPIRAM);
	//heap_caps_print_heap_info(MALLOC_CAP_DMA);
	//heap_caps_print_heap_info(MALLOC_CAP_EXEC);
	//heap_caps_print_heap_info(MALLOC_CAP_8BIT);
	//heap_caps_print_heap_info(MALLOC_CAP_32BIT);
	//heap_caps_check_integrity_all(true); // prints errors if heaps are corrupt
}

// print freertos tasks table
static int l_cmd_tasks(int argc, char **argv) {
	l_vTaskList();

	fputs("\n", stdout);

	// TODO: re-implement vTaskGetRunTimeStats or integrate with l_vTaskList

	fputs("vTaskGetRunTimeStats. Table is valid until counters overflow (when that happens? don't know)\n", stdout);
	fputs("Runtime_Counter: don't know units.\n", stdout);
	fputs("-----------------------------------------------------------------\n", stdout);
	fputs("Task_Name\tRuntime_Counter\tAs_Percentage\n", stdout);
	fputs("-----------------------------------------------------------------\n", stdout);
	const size_t bytes_per_task = 40; // almost random.. assume no more bytes per table line
	char task_list_buffer[uxTaskGetNumberOfTasks() * bytes_per_task];
	vTaskGetRunTimeStats(task_list_buffer);
	fputs(task_list_buffer, stdout);
	fputs("-----------------------------------------------------------------\n", stdout);

	fputs("\n", stdout);

	return ESP_OK;
}

//void l_console_register_commands(const char *command_name, const char *help, const char *hint, esp_console_cmd_func_t func, void *argtable) {
static void l_console_register_commands() {
	esp_console_cmd_t cmd = {};
	cmd.command = "tasks";
	cmd.help = "print freertos tasks table",
	cmd.func = l_cmd_tasks;
	ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));

//	{
//		.command = "tasks",
//		.help = nullptr,
//		.hint = nullptr,
//		.func = l_cmd_tasks,
//		.argtable = nullptr
//	};

//	cmd.command = "param";
//	cmd.help = "read/write parameters. without any args - print out param table. with on arg, print out single param. with two args, set value to the param.",
//	cmd.func = l_cmd_param;
//	ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));

//	ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
//	ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

//// Takes a packet without the framing zeroes.
static void l_rx_packet(u8* data, u8 len) {

	//char tmp[80]; ESP_LOGI(TAG, "rx deframed packet %i: %s", len, g_buf2hexstr(data, len, tmp, sizeof(tmp)));

	u8 dst[L_MAX_DECODED_PACKET_SIZE];

	i32 decoded_len = l_cobs_decode_crc(data, len, dst);

	if (decoded_len < 1 + SERIALCOMM_CRC_BYTES) { // crc error, decode error, or no payload.
		if (decoded_len == -2) {
			g_debug_console_rx_crc_errors++;
			ESP_LOGE(TAG, "rx crc error %i:", len);
		} else {
			g_debug_console_rx_packets_broken++;
			ESP_LOGE(TAG, "rx broken packet %i:", len);
		}
		ESP_LOG_BUFFER_HEX_LEVEL(TAG, (u8*)data, len, ESP_LOG_ERROR);
		return;
	}

	//ESP_LOGI(TAG, "rx decoded packet %i:", decoded_len);
	//ESP_LOG_BUFFER_HEX_LEVEL(TAG, dst, decoded_len, ESP_LOG_INFO);

	//g_console_rx_packet(dst, decoded_len - SERIALCOMM_CRC_BYTES); // send to upper layers
}

// // command has to end with newline. /n ? or /r/n? /n/r?
// NB! command shall end with zero-termination! no newline char is necessary.
static void l_rx_console_command(const char* buf, u32 len) {
	int ret;

	esp_err_t err = esp_console_run(buf, &ret);

	if (err == ESP_ERR_NOT_FOUND) {
		printf("Unrecognized command (len %i): '%s'\n", len, buf);
	} else if (err == ESP_ERR_INVALID_ARG) {
		// command was empty
	} else if (err == ESP_OK && ret != ESP_OK) {
		printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(ret));
	} else if (err != ESP_OK) {
		printf("Internal error: %s\n", esp_err_to_name(err));
	}
}

#define ST_COBS_WAITING_START_1  0
#define ST_COBS_WAITING_START_2  1
#define ST_COBS_RECEIVING_PACKET 2

static int l_cobs_state;

// for use when the packet-delimiting zero has already been received.
static inline void l_cobs_restart_rx___wait_for_nonzero() {
	l_cobs_state = ST_COBS_WAITING_START_2;
}

static inline void l_cobs_rx_byte(u8 byte) {
	// RX (COBS-encoded)
	//
	// Find packets between zeroes.
	// Matching stuff gets written to local static rx_buf and sent to l_recv_packet(..). (without the framing zeros)

	static int l_cobs_state;
	static u8  rx_buf[l_cobs_encode_crc_with_frame_overhead(L_MAX_DECODED_PACKET_SIZE)];
	static u32 rx_buf_size;

	if (l_cobs_state == ST_COBS_WAITING_START_1) {
		// This l_cobs_state eats the half-packet in case we powered on during a packet transfer.
		// (This logic should be time-based, because when the sender gets shut down during transfer,
		// we'll be in exactly the same situation we're trying to prevent here.)
		if (byte == 0) l_cobs_state = ST_COBS_WAITING_START_2;
	} else if (l_cobs_state == ST_COBS_WAITING_START_2) {
		// This l_cobs_state eats consecutive zeroes between packets. Go to next l_cobs_state only if byte is not zero.
		if (byte != 0) {
			l_cobs_state = ST_COBS_RECEIVING_PACKET;
			rx_buf_size = 0;
		}
	}

	if (l_cobs_state == ST_COBS_RECEIVING_PACKET) {
		if (byte == 0) {
			// packet end
			l_rx_packet(rx_buf, rx_buf_size);
			l_cobs_state = ST_COBS_WAITING_START_2;
		} else {
			if (rx_buf_size >= sizeof(rx_buf)) {
				g_debug_console_rx_packets_broken++;
				ESP_LOGE(TAG, "broken packet in deframer. num %i", g_debug_console_rx_packets_broken);
				l_cobs_state = ST_COBS_WAITING_START_1; // new byte won't fit anymore
			} else {
				rx_buf[rx_buf_size++] = byte;
			}
		}
	}
}


#define ST_RXMODE_HUMAN_INTERACTIVE 0  // input from terminal. typing commands.
#define ST_RXMODE_MACHINE_PACKETS   1  // input from cobs-encoded packets

static int l_rxmode_state;

// Bytes from the user are given to this function one-by-one. Routes the bytes to interactive terminal logic, or
// to cobs-encoded packet decoder.
//
//   if byte == 0: enter cobs-packets receiving state.
//   if previous byte is older than 0.5 seconds, return to human-interactive state.
//   if there has been no new bytes for 10 minutes, clear the console input-line buffer.
//   (cobs-encoded packets can be sent anytime. interactive command buf will continue from were it was left)
//
static inline void l_rx_byte(u8 byte) {

	static u8  cmd_buf[L_CONSOLE_CMD_MAX_LEN];
	static u32 cmd_buf_size;

	static i64 last_byte_time;
	i64 byte_time = esp_timer_get_time();

	if (l_rxmode_state == ST_RXMODE_HUMAN_INTERACTIVE) {

		// if 10 minutes has passed since last byte, clear the command buf.
		if (byte_time - last_byte_time > 1000000*60*10)
			cmd_buf_size = 0;

	} else { // ST_RXMODE_MACHINE_PACKETS

		// if time from last byte is > 0.5 s, return to human-interactive mode. any half-packet
		// already in buf will be lost.
		if (byte_time - last_byte_time > 500000)
			l_rxmode_state = ST_RXMODE_HUMAN_INTERACTIVE;
	}

	if (l_rxmode_state == ST_RXMODE_HUMAN_INTERACTIVE) {

		if (byte == 0) { // cobs-encoded packet delimiter from terminal? don't think so - enter packet receiving mode.

			l_rxmode_state = ST_RXMODE_MACHINE_PACKETS;
			l_cobs_restart_rx___wait_for_nonzero();

		} else if (byte == 0x1b) { // esc button

			cmd_buf_size = 0; // clear any partial command in buf
			// TODO: for some reason we need double newlines for idf.py monitor. putty works like it should, with one.
			uart_write_bytes(L_UART_NUM, "\r\n\r\n" PROMPT, l_prompt_len+4);
			//uart_write_bytes(L_UART_NUM, "\r\n" PROMPT, l_prompt_len+2);

		} else {
			if (cmd_buf_size >= sizeof(cmd_buf)) {
				ESP_LOGE(TAG, "console command len overflow (max is %u bytes)! press esc to clear buf.", (u32)sizeof(cmd_buf));
				cmd_buf_size = 0;
			} else {
				// \r CR 0x0d (mac os <= 9. AND: Minicom, screen, putty, idf_monitor send CR when ENTER key is pressed)
				// \n LF 0x0a (linux, unix, macosx)
				// CRLF (windows)

				// ok this code assumes we get either CR or LF, but NEVER CRLF.

				//ESP_LOGE(TAG, "c 0x%02x", byte);

				if (byte == 0x0a || byte == 0x0d) { // enter/return button

					// TODO: for some reason we need double newlines for idf.py monitor. putty works like it should, with one.
					uart_write_bytes(L_UART_NUM, "\r\n\r\n", 4);
					//uart_write_bytes(L_UART_NUM, "\r\n", 2);

					if (cmd_buf_size == 0) {
						uart_write_bytes(L_UART_NUM, PROMPT, l_prompt_len);
					} else {
						cmd_buf[cmd_buf_size] = 0;

						l_rx_console_command((char*)cmd_buf, cmd_buf_size);

						//fflush(stdout);
						uart_write_bytes(L_UART_NUM, PROMPT, l_prompt_len);
					}
					cmd_buf_size = 0;

				} else if (byte >= 0x20 && byte < 0x7f) { // filter out all control characters

					cmd_buf[cmd_buf_size++] = byte;
					uart_write_bytes(L_UART_NUM, (const char*)&byte, 1); // echo the received char
				}
			}
		}

	} else { // ST_RXMODE_MACHINE_PACKETS

		l_cobs_rx_byte(byte);
	}

	last_byte_time = byte_time;
}

[[noreturn]] static void l_uart_event_task(void *pvParameters) {
	uart_event_t event;
	u8 rxbuf[32];

	while (1) {
		if (xQueueReceive(l_uart_queue, (void*)&event, portMAX_DELAY)) {
			//uart_write_bytes(L_UART_NUM, "event", 5);

			switch (event.type) {
			case UART_DATA: {
				size_t len = g_min(event.size, sizeof(rxbuf));
				int rx_numbytes = uart_read_bytes(L_UART_NUM, rxbuf, len, portMAX_DELAY);
				if (rx_numbytes <= 0) {
					ESP_LOGE(TAG, "uart_read_bytes err %i", rx_numbytes);
					break;
				}

				//char tmp[80]; ESP_LOGI(TAG, "CONSOLE_UART rx %d %d: %s", event.size, rx_numbytes, g_buf2hexstr(rxbuf, event.size, tmp, sizeof(tmp)));
				//uart_write_bytes(L_UART_NUM, (const char*)rxbuf, rx_numbytes); // temp echo

				for (int i = 0; i < rx_numbytes; i++)
					l_rx_byte(rxbuf[i]);

				#ifdef USE_LIGHT_SLEEP
					// disable cpu sleep for a while to enable keyboard-interacti comm with human
					// speed. or the cpu would go to sleep and bytes would not be received.
					l_pm_lock_poke_timer();
				#endif
				break;
			}
			case UART_FIFO_OVF:
			case UART_BUFFER_FULL: {
				uart_flush_input(L_UART_NUM);
				xQueueReset(l_uart_queue);
				break;
			}
			default: break;
			}
		}
	}
}


#ifdef USE_LIGHT_SLEEP

	// To cut down on esp_pm_lock_acquire and esp_timer_start_once calls, we just remember the
	// l_pm_lock_release_timer_restart flag. and when the timer fires, we check the flag and restart
	// the timer if flag is true. This means that the real timeout until cpu sleep is re-enabled is
	// between PM_LOCK_RELEASE_TIMEOUT_US and PM_LOCK_RELEASE_TIMEOUT_US * 2.

	static inline void l_pm_lock_poke_timer() {
		xSemaphoreTake(l_pm_lock_timer_mutex, portMAX_DELAY);
		if (!l_pm_lock_acquired) {
			ESP_LOGE(TAG, "disabling cpu sleep for %i seconds", PM_LOCK_RELEASE_TIMEOUT_US / 1000000);
			ESP_ERROR_CHECK(esp_pm_lock_acquire(l_uart0_rx_pm_lock));
			l_pm_lock_acquired = true;
			ESP_ERROR_CHECK(esp_timer_start_once(l_pm_lock_release_timer_handle, PM_LOCK_RELEASE_TIMEOUT_US));
		} else {
			l_pm_lock_release_timer_restart = true;
		}
		xSemaphoreGive(l_pm_lock_timer_mutex);
	}

	static void l_pm_lock_release_timer_callback(void* arg) {
		xSemaphoreTake(l_pm_lock_timer_mutex, portMAX_DELAY);

		assert(l_pm_lock_acquired);

		if (l_pm_lock_release_timer_restart) {
			ESP_ERROR_CHECK(esp_timer_start_once(l_pm_lock_release_timer_handle, PM_LOCK_RELEASE_TIMEOUT_US));
			l_pm_lock_release_timer_restart = false;
		} else {
			ESP_ERROR_CHECK(esp_pm_lock_release(l_uart0_rx_pm_lock));
			l_pm_lock_acquired = false;
		}

		xSemaphoreGive(l_pm_lock_timer_mutex);
	}

#endif
