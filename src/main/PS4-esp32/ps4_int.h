#pragma once

#include "sdkconfig.h"

// Check the configured bluetooth mode
#ifdef CONFIG_BTDM_CONTROLLER_MODE_BTDM
	#warning Using bluetooth dual mode. ESP_BT_MODE_BTDM
	#define BT_MODE ESP_BT_MODE_BTDM
#elif defined CONFIG_BTDM_CONTROLLER_MODE_BR_EDR_ONLY
	#warning Using bluetooth clasic mode. ESP_BT_MODE_CLASSIC_BT
	#define BT_MODE ESP_BT_MODE_CLASSIC_BT
#else
	#error "The selected Bluetooth controller mode is not supported by the ESP32-PS4 module"
#endif

#include <esp_idf_version.h>

// Macro to convert IDF version number into an integer
#define ESP_IDF_VERSION_VAL(major, minor, patch) ((major << 16) | (minor << 8) | (patch))

// Current IDF version, as an integer
#define ESP_IDF_VERSION  ESP_IDF_VERSION_VAL(ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH)

// Size of the output report buffer for the Dualshock and Navigation controllers
#define PS4_SEND_BUFFER_SIZE 77
#define PS4_HID_BUFFER_SIZE  50

// ********************************************************************************************************************
// S H A R E D   T Y P E S

enum hid_cmd_code {
	hid_cmd_code_set_report   = 0x50,
	hid_cmd_code_type_output  = 0x02,
	hid_cmd_code_type_feature = 0x03
};

enum hid_cmd_identifier {
	hid_cmd_identifier_ps4_enable  = 0xF4,
	hid_cmd_identifier_ps4_control = 0x11
};

typedef struct {
	uint8_t code;
	uint8_t identifier;
	uint8_t data[PS4_SEND_BUFFER_SIZE];
} hid_cmd_t;

enum ps4_control_packet_index {
	ps4_control_packet_index_small_rumble   = 5,
	ps4_control_packet_index_large_rumble   = 6,

	ps4_control_packet_index_red            = 7,
	ps4_control_packet_index_green          = 8,
	ps4_control_packet_index_blue           = 9,

	ps4_control_packet_index_flash_on_time  = 10,
	ps4_control_packet_index_flash_off_time = 11
};

// ********************************************************************************************************************
// C A L L B A C K   F U N C T I O N S

void ps4ConnectEvent(uint8_t isConnected);
void ps4PacketEvent(ps4_t ps4, ps4_event_t event);

// ********************************************************************************************************************
// P A R S E R   F U N C T I O N S

void parsePacket(uint8_t* packet);

// ********************************************************************************************************************
// S P P   F U N C T I O N S

void sppInit();

// ********************************************************************************************************************
// G A P   F U N C T I O N S

void ps4_l2cap_init_services();
void ps4_l2cap_deinit_services();
void ps4_l2cap_send_hid(hid_cmd_t* hid_cmd, uint8_t len);
