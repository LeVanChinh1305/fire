#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"
#include <esp_ble_mesh_defs.h>

#define DF_MESH_CID_ESP                 DF_MESH_VND_COMPANY_ID      /* 0x8888 */
#define DF_MESH_VND_MODEL_ID_CLIENT     0x0001

#define DF_MESH_VND_OP_SET      ESP_BLE_MESH_MODEL_OP_3(0x01, DF_MESH_CID_ESP)
#define DF_MESH_VND_OP_GET      ESP_BLE_MESH_MODEL_OP_3(0x02, DF_MESH_CID_ESP)
#define DF_MESH_VND_OP_STATUS   ESP_BLE_MESH_MODEL_OP_3(0x03, DF_MESH_CID_ESP)

#define DF_CMD_FIRE_ALARM_NORMAL        0x00
#define DF_CMD_FIRE_ALARM_TRIGGER       0x01
#define DF_CMD_FIRE_ALARM_MUTE          0x02

typedef struct __attribute__((packed)) {
    uint8_t  u8Cmd;
    uint16_t u16PacketId;
    int16_t  s16Temperature;
    uint16_t u16SmokeVal;
    uint8_t  u8AlarmState;
    char     acDeviceTag[16];
} st_vendor_custom_data_t;