#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "stack/gap_api.h"
#include "stack/bt_types.h"
#include "stack/l2c_api.h"
#include "osi/allocator.h"

#include "ps4.h"
#include "ps4_int.h"

#define PS4_TAG "PS4_L2CAP"


static void ps4_l2cap_init_service(const char* name, uint16_t psm, uint8_t security_id);
static void ps4_l2cap_deinit_service(const char* name, uint16_t psm);
static void ps4_l2cap_connect_ind_cback(BD_ADDR bd_addr, uint16_t l2cap_cid, uint16_t psm, uint8_t l2cap_id);
static void ps4_l2cap_connect_cfm_cback(uint16_t l2cap_cid, uint16_t result);
static void ps4_l2cap_config_ind_cback(uint16_t l2cap_cid, tL2CAP_CFG_INFO* p_cfg);
static void ps4_l2cap_config_cfm_cback(uint16_t l2cap_cid, tL2CAP_CFG_INFO* p_cfg);
static void ps4_l2cap_disconnect_ind_cback(uint16_t l2cap_cid, bool ack_needed);
static void ps4_l2cap_disconnect_cfm_cback(uint16_t l2cap_cid, uint16_t result);
static void ps4_l2cap_data_ind_cback(uint16_t l2cap_cid, BT_HDR* p_msg);
static void ps4_l2cap_congest_cback(uint16_t cid, bool congested);


static const tL2CAP_APPL_INFO dyn_info = {
	ps4_l2cap_connect_ind_cback,
	ps4_l2cap_connect_cfm_cback,
	NULL,
	ps4_l2cap_config_ind_cback,
	ps4_l2cap_config_cfm_cback,
	ps4_l2cap_disconnect_ind_cback,
	ps4_l2cap_disconnect_cfm_cback,
	NULL,
	ps4_l2cap_data_ind_cback,
	ps4_l2cap_congest_cback,
	NULL
};

static tL2CAP_CFG_INFO ps4_cfg_info;

static bool is_connected = false;
static uint16_t l2cap_control_channel = 0;
static uint16_t l2cap_interrupt_channel = 0;


// ********************************************************************************************************************
//                     P U B L I C    F U N C T I O N S
// ********************************************************************************************************************


// This function initialises the required L2CAP services.
void ps4_l2cap_init_services() {
	ps4_l2cap_init_service("PS4-HIDC", HID_PSM_CONTROL, BTM_SEC_SERVICE_FIRST_EMPTY);
	ps4_l2cap_init_service("PS4-HIDI", HID_PSM_INTERRUPT, BTM_SEC_SERVICE_FIRST_EMPTY + 1);
}

// This function deinitialises the required L2CAP services.
void ps4_l2cap_deinit_services() {
	ps4_l2cap_deinit_service("PS4-HIDC", HID_PSM_CONTROL);
	ps4_l2cap_deinit_service("PS4-HIDI", HID_PSM_INTERRUPT);
}


// This function sends the HID command using the L2CAP service.
void ps4_l2cap_send_hid(hid_cmd_t* hid_cmd, uint8_t len) {
	uint8_t result;
	BT_HDR* p_buf;

	p_buf = (BT_HDR*) osi_malloc(BT_DEFAULT_BUFFER_SIZE);

	if (!p_buf)
		ESP_LOGE(PS4_TAG, "[%s] allocating buffer for sending the command failed", __func__);

	p_buf->len = len + (sizeof(*hid_cmd) - sizeof(hid_cmd->data));
	p_buf->offset = L2CAP_MIN_OFFSET;

	memcpy((uint8_t*) (p_buf + 1) + p_buf->offset, (uint8_t*) hid_cmd, p_buf->len);

	if (l2cap_control_channel == 0)
		ESP_LOGE(PS4_TAG, "[%s] l2cap_control_channel not initialized.", __func__);

	result = L2CA_DataWrite(l2cap_control_channel, p_buf);

	if (result == L2CAP_DW_SUCCESS)
		ESP_LOGI(PS4_TAG, "[%s] sending command: success", __func__);

	if (result == L2CAP_DW_CONGESTED)
		ESP_LOGW(PS4_TAG, "[%s] sending command: congested", __func__);

	if (result == L2CAP_DW_FAILED)
		ESP_LOGE(PS4_TAG, "[%s] sending command: failed", __func__);
}


// ********************************************************************************************************************
//                     L O C A L    F U N C T I O N S
// ********************************************************************************************************************


// Convert 6 bytes to address string "01:02:03:04:ab:cd". Not threadsafe, as it uses an internal buffer for the result.
const char* bd_addr_str(BD_ADDR bd_addr) {
	static char dst[18];
	const char* hexes = "0123456789abcdef";
	uint8_t* src = (uint8_t*)&bd_addr;
	int idst = 0;

	while (idst < 3 * 6) {
		uint8_t b = *src++;
		dst[idst++] = hexes[b >> 4];
		dst[idst++] = hexes[b & 0xf];
		dst[idst++] = ':';
	}
	dst[17] = 0;

	return dst;
}

// This registers the specified bluetooth service in order to listen for incoming connections.
static void ps4_l2cap_init_service(const char* name, uint16_t psm, uint8_t security_id) {
	// Register the PSM for incoming connections
	if (!L2CA_Register(psm, (tL2CAP_APPL_INFO*) &dyn_info)) {
		ESP_LOGE(PS4_TAG, "%s Registering service %s failed", __func__, name);
		return;
	}

	// Register with the Security Manager for our specific security level (none)
	if (!BTM_SetSecurityLevel(false, name, security_id, 0, psm, 0, 0)) {
		ESP_LOGE (PS4_TAG, "%s Registering security service %s failed", __func__, name);
		return;
	}

	ESP_LOGI(PS4_TAG, "[%s] Service %s Initialized", __func__, name);
}

// This deregisters the specified bluetooth service.
static void ps4_l2cap_deinit_service(const char* name, uint16_t psm) {
	// Deregister the PSM from incoming connections
	L2CA_Deregister(psm);
	ESP_LOGI(PS4_TAG, "[%s] Service %s Deinitialized", __func__, name);
}

// This the L2CAP inbound connection indication callback function.
static void ps4_l2cap_connect_ind_cback(BD_ADDR bd_addr, uint16_t l2cap_cid, uint16_t psm, uint8_t l2cap_id) {
	ESP_LOGI(PS4_TAG, "[%s] bd_addr: %s\n  l2cap_cid: 0x%02x\n  psm: %d\n  id: %d", __func__, bd_addr_str(bd_addr), l2cap_cid,
	         psm, l2cap_id);

	// Send connection pending response to the L2CAP layer.
	L2CA_CONNECT_RSP(bd_addr, l2cap_id, l2cap_cid, L2CAP_CONN_PENDING, L2CAP_CONN_PENDING, NULL, NULL);

	// Send response to the L2CAP layer.
	L2CA_CONNECT_RSP(bd_addr, l2cap_id, l2cap_cid, L2CAP_CONN_OK, L2CAP_CONN_OK, NULL, NULL);

	// Send a Configuration Request.
	L2CA_CONFIG_REQ(l2cap_cid, &ps4_cfg_info);

	if (psm == HID_PSM_CONTROL) {
		l2cap_control_channel = l2cap_cid;
	} else if (psm == HID_PSM_INTERRUPT) {
		l2cap_interrupt_channel = l2cap_cid;
	}
}

// This is the L2CAP connect confirmation callback function.
static void ps4_l2cap_connect_cfm_cback(uint16_t l2cap_cid, uint16_t result) {
	ESP_LOGI(PS4_TAG, "[%s] l2cap_cid: 0x%02x\n  result: %d", __func__, l2cap_cid, result);
}

// This is the L2CAP config confirmation callback function.
void ps4_l2cap_config_cfm_cback(uint16_t l2cap_cid, tL2CAP_CFG_INFO* p_cfg) {
	ESP_LOGI(PS4_TAG, "[%s] l2cap_cid: 0x%02x\n  p_cfg->result: %d", __func__, l2cap_cid, p_cfg->result);

	// The PS4 controller is connected after receiving the second config confirmation
	bool prev_is_connected = is_connected;
	is_connected = l2cap_cid == l2cap_interrupt_channel;

	if (prev_is_connected != is_connected)
		ps4ConnectEvent(is_connected);
}

// This is the L2CAP config indication callback function.
void ps4_l2cap_config_ind_cback(uint16_t l2cap_cid, tL2CAP_CFG_INFO* p_cfg) {
	ESP_LOGI(PS4_TAG, "[%s] l2cap_cid: 0x%02x\n  p_cfg->result: %d\n  p_cfg->mtu_present: %d\n  p_cfg->mtu: %d",
	         __func__, l2cap_cid, p_cfg->result, p_cfg->mtu_present, p_cfg->mtu);

	p_cfg->result = L2CAP_CFG_OK;

	L2CA_ConfigRsp(l2cap_cid, p_cfg);
}

// This is the L2CAP disconnect indication callback function.
void ps4_l2cap_disconnect_ind_cback(uint16_t l2cap_cid, bool ack_needed) {
	ESP_LOGI(PS4_TAG, "[%s] l2cap_cid: 0x%02x\n  ack_needed: %d", __func__, l2cap_cid, ack_needed);
	is_connected = false;

	if (ack_needed)
		L2CA_DisconnectRsp(l2cap_cid);

	ps4ConnectEvent(is_connected);
}

// This is the L2CAP disconnect confirm callback function.
static void ps4_l2cap_disconnect_cfm_cback(uint16_t l2cap_cid, uint16_t result) {
	ESP_LOGI(PS4_TAG, "[%s] l2cap_cid: 0x%02x\n  result: %d", __func__, l2cap_cid, result);
}

// This is the L2CAP data indication callback function.
static void ps4_l2cap_data_ind_cback(uint16_t l2cap_cid, BT_HDR* p_buf) {
	if (p_buf->len > 2)
		parsePacket(p_buf->data);

	osi_free(p_buf);
}

// This is the L2CAP congestion callback function.
static void ps4_l2cap_congest_cback(uint16_t l2cap_cid, bool congested) {
	ESP_LOGI(PS4_TAG, "[%s] l2cap_cid: 0x%02x\n  congested: %d", __func__, l2cap_cid, congested);
}