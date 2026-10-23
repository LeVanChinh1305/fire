#include "app_blemesh.h"
#include "app_config.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_health_model_api.h"
#include "esp_ble_mesh_lighting_model_api.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "FIRE_MESH";

#define MESH_INFO_MUTEX_NAME    "FIRE_MESH_MUTEX"
#define DF_LOCK_TIMEOUT_MS      1000

#ifndef ROLE_NODE
#define ROLE_NODE 0
#endif

typedef enum {
    MESH_EVT_ENABLED = 0,
    MESH_EVT_PROV_COMPLETE,
    MESH_EVT_PROV_RESET,
    MESH_EVT_APPKEY_ADD,
    MESH_EVT_APP_BIND,
    MESH_EVT_SUB_ADD,
    MESH_EVT_SUB_DEL,
    MESH_EVT_ONOFF_SET,
    MESH_EVT_VENDOR_MSG,
    MESH_EVT_LOCAL_APPKEY_ADD,
    MESH_EVT_LOCAL_APP_BIND,
    MESH_EVT_LOCAL_SUB_ADD,
    MESH_EVT_LOCAL_SUB_DEL
} en_mesh_evt_t;

typedef struct {
    en_mesh_evt_t eType;
    int64_t       s64TimeUs;
    uint16_t      u16Src;
    uint16_t      u16Dst;
    uint16_t      u16Param;
    uint16_t      u16ModelId;
    int           iErrCode;
    uint8_t       u8OnOff;
    uint8_t       u8Ttl;
    int8_t        s8Rssi;
} st_mesh_evt_t;

typedef struct {
    bool     bProvisioned;
    uint16_t u16Unicast;
    uint16_t u16NetIdx;
    uint16_t u16AppIdx;
    bool     bAppBound;
    uint16_t u16Group;
    uint8_t  u8AlarmState;
    uint32_t u32BootCount;
    uint32_t u32RxCount;
    uint32_t u32TxCount;
    uint16_t u16LastSrc;
    uint16_t u16LastDst;
} st_mesh_info_t;

typedef struct {
    int64_t  s64TimeUs;
    uint16_t u16Src;
    uint16_t u16Dst;
    uint8_t  u8Cmd;
    uint8_t  u8State;
    uint8_t  u8Ttl;
    int8_t   s8Rssi;
} st_mesh_log_t;

static st_mesh_info_t    g_stInfo;
static st_mesh_log_t     g_astLog[DF_MESH_LOG_DEPTH];
static uint32_t          g_u32LogHead       = 0;
static uint32_t          g_u32LogCount      = 0;
static SemaphoreHandle_t g_hMeshMutex       = NULL;
static QueueHandle_t     g_hMeshEvtQ        = NULL;
static TaskHandle_t      g_hMeshTask        = NULL;
static bool              g_bMeshInitialized = false;
static uint32_t          g_u32DropEvt       = 0;

static const uint8_t HOMI_APP_KEY_0[16] = {
    0x16, 0xE7, 0xC9, 0x34, 0xEB, 0x3F, 0x5A, 0x8B,
    0xDB, 0x63, 0x2A, 0x09, 0x6A, 0xA8, 0x2C, 0xAC,
};

static const uint8_t HOMI_APP_KEY_1[16] = {
    0xF0, 0x7E, 0xE1, 0xB4, 0xEF, 0x3C, 0xFE, 0x00,
    0xD7, 0x08, 0x43, 0xF6, 0x3D, 0x0A, 0xBC, 0xAB,
};

static const uint8_t HOMI_APP_KEY_2[16] = {
    0xB3, 0xAA, 0x31, 0xD0, 0x7A, 0xFB, 0x8D, 0x21,
    0x54, 0x2D, 0x85, 0x35, 0x33, 0xE6, 0xC3, 0xDD,
};

static bool g_bGatewayJoined = false;
static uint8_t g_u8AppKeyReadyMask;
static uint8_t g_u8BindPendingCount;
static uint8_t g_u8BindFailedCount;
static uint8_t g_u8GroupPendingCount;
static bool g_bGatewayConfigStarted;
static bool g_bGroupConfigFailed;
static uint16_t g_u16ConfiguredGroup = DF_MESH_GROUP_FIRE_ALARM;
static uint16_t g_u16ProvisionedNetIdx = DF_MESH_HOMI_NET_KEY_IDX;

#define DF_MAX_SCANNED_DEVICES 32

typedef struct {
    uint8_t  au8Mac[6];
    uint8_t  u8AddrType;
    uint8_t  au8Uuid[16];
    bool     bHasUuid;
    char     acDeviceType[32];
    int8_t   s8Rssi;
    uint16_t u16OobInfo;
    uint32_t u32PktCount;
    int64_t  s64FirstSeenUs;
    int64_t  s64LastSeenUs;
} st_mesh_scanned_dev_t;

static st_mesh_scanned_dev_t g_astScannedDevs[DF_MAX_SCANNED_DEVICES];
static uint32_t              g_u32ScannedCount  = 0;
static SemaphoreHandle_t     g_hScanMutex       = NULL;
static TaskHandle_t          g_hScanPrintTask   = NULL;

static void app_mesh_ScannerTask(void *pvParameters);
static int32_t app_mesh_GatewayJoinNetwork(uint16_t u16NetIdx);
static int32_t app_mesh_SubscribeGroup(uint16_t u16GroupAddr, bool bPersist);

static uint8_t g_au8DevUuid[16] = { 0x32, 0x10 };

static esp_ble_mesh_prov_t g_stProv = {
    .uuid = g_au8DevUuid,
};

static esp_ble_mesh_cfg_srv_t g_stCfgSrv = {
#if DF_MESH_RELAY
    .relay            = ESP_BLE_MESH_RELAY_ENABLED,
#else
    .relay            = ESP_BLE_MESH_RELAY_DISABLED,
#endif
    .beacon           = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy       = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy       = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .friend_state     = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
    .default_ttl      = 7,
    .net_transmit     = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stOnoffPub, 2 + 3, ROLE_NODE);

static esp_ble_mesh_gen_onoff_srv_t g_stOnoffSrv = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
};

static esp_ble_mesh_client_t g_stCfgCli;
static esp_ble_mesh_client_t g_stOnoffCli;
static esp_ble_mesh_client_t g_stHealthCli;
static esp_ble_mesh_client_t g_stLevelCli;
static esp_ble_mesh_client_t g_stLightnessCli;
static esp_ble_mesh_client_t g_stCtlCli;

static esp_ble_mesh_model_t g_astRootModels[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&g_stCfgSrv),
    ESP_BLE_MESH_MODEL_CFG_CLI(&g_stCfgCli),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&g_stOnoffPub, &g_stOnoffSrv),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(&g_stOnoffPub, &g_stOnoffCli),
    ESP_BLE_MESH_MODEL_HEALTH_CLI(&g_stHealthCli),
    ESP_BLE_MESH_MODEL_GEN_LEVEL_CLI(&g_stOnoffPub, &g_stLevelCli),
    ESP_BLE_MESH_MODEL_LIGHT_LIGHTNESS_CLI(&g_stOnoffPub, &g_stLightnessCli),
    ESP_BLE_MESH_MODEL_LIGHT_CTL_CLI(&g_stOnoffPub, &g_stCtlCli),
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stVndSrvPub, 3 + sizeof(st_vendor_custom_data_t), ROLE_NODE);

#define VND_OP(n)     ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_MODEL_OP_3((n), DF_MESH_VND_COMPANY_ID), 0)
#define VND_OPS_8(b)  VND_OP((b)+0), VND_OP((b)+1), VND_OP((b)+2), VND_OP((b)+3), \
                      VND_OP((b)+4), VND_OP((b)+5), VND_OP((b)+6), VND_OP((b)+7)

static esp_ble_mesh_model_op_t g_astVndServerOp[] = {
    VND_OPS_8(0x00), VND_OPS_8(0x08), VND_OPS_8(0x10), VND_OPS_8(0x18),
    VND_OPS_8(0x20), VND_OPS_8(0x28), VND_OPS_8(0x30), VND_OPS_8(0x38),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t g_astVndModelsServer[] = {
    ESP_BLE_MESH_VENDOR_MODEL(
        DF_MESH_CID_ESP,
        DF_MESH_VND_MODEL_ID_SERVER,
        g_astVndServerOp,
        &g_stVndSrvPub,
        NULL
    ),
};

static esp_ble_mesh_elem_t g_astElements[] = {
    ESP_BLE_MESH_ELEMENT(0, g_astRootModels, g_astVndModelsServer),
};

static esp_ble_mesh_comp_t g_stComp = {
    .cid           = DF_MESH_COMPANY_ID,
    .element_count = ARRAY_SIZE(g_astElements),
    .elements      = g_astElements,
};

static void app_mesh_PostEvt(const st_mesh_evt_t *pstEvt)
{
    if (g_hMeshEvtQ == NULL) return;
    if (xQueueSend(g_hMeshEvtQ, pstEvt, 0) != pdTRUE) {
        g_u32DropEvt++;
    }
}

static void app_mesh_AppendLog(uint16_t u16Src, uint16_t u16Dst, uint8_t u8Cmd, uint8_t u8State, uint8_t u8Ttl, int8_t s8Rssi)
{
    APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
    if (!stGuard.bLocked) return;

    g_astLog[g_u32LogHead].s64TimeUs = esp_timer_get_time();
    g_astLog[g_u32LogHead].u16Src    = u16Src;
    g_astLog[g_u32LogHead].u16Dst    = u16Dst;
    g_astLog[g_u32LogHead].u8Cmd     = u8Cmd;
    g_astLog[g_u32LogHead].u8State   = u8State;
    g_astLog[g_u32LogHead].u8Ttl     = u8Ttl;
    g_astLog[g_u32LogHead].s8Rssi    = s8Rssi;

    g_u32LogHead = (g_u32LogHead + 1) % DF_MESH_LOG_DEPTH;
    if (g_u32LogCount < DF_MESH_LOG_DEPTH) {
        g_u32LogCount++;
    }
}

static void app_mesh_HealthCliCb(esp_ble_mesh_health_client_cb_event_t eEvent,
                                 esp_ble_mesh_health_client_cb_param_t *pstParam)
{
    if (pstParam == NULL) return;

    uint16_t u16Src = 0, u16Dst = 0;
    if (pstParam->params) {
        u16Src = pstParam->params->ctx.addr;
        u16Dst = pstParam->params->ctx.recv_dst;
    }

    switch (eEvent) {
    case ESP_BLE_MESH_HEALTH_CLIENT_GET_STATE_EVT:
    case ESP_BLE_MESH_HEALTH_CLIENT_SET_STATE_EVT:
    case ESP_BLE_MESH_HEALTH_CLIENT_PUBLISH_EVT: {
        uint32_t op = pstParam->params ? pstParam->params->opcode : 0;
        LREP(TAG, "[HEALTH RX] 0x%04X -> 0x%04X | op=0x%04lX | err=%d",
             u16Src, u16Dst, (unsigned long)op, pstParam->error_code);

        if (pstParam->error_code != ESP_OK) break;

        if (op == ESP_BLE_MESH_MODEL_OP_HEALTH_FAULT_STATUS ||
            op == ESP_BLE_MESH_MODEL_OP_HEALTH_CURRENT_STATUS) {
            uint8_t  test_id  = pstParam->status_cb.fault_status.test_id;
            uint16_t company  = pstParam->status_cb.fault_status.company_id;
            struct net_buf_simple *pFaults = pstParam->status_cb.fault_status.fault_array;

            printf("\n[HEALTH] Fault Status tu 0x%04X\n", u16Src);
            printf("  Test ID     : 0x%02X\n", test_id);
            printf("  Company ID  : 0x%04X\n", company);

            if (pFaults && pFaults->len > 0) {
                printf("  So loi      : %u\n  Ma loi      :", (unsigned)pFaults->len);
                for (uint16_t i = 0; i < pFaults->len; i++) {
                    printf(" 0x%02X", pFaults->data[i]);
                }
                printf("\n");
            } else {
                printf("  (Khong co loi)\n");
            }
            printf("\n");
        } else if (op == ESP_BLE_MESH_MODEL_OP_ATTENTION_STATUS) {
            printf("\n[HEALTH] Attention Status tu 0x%04X: %u giay\n\n",
                   u16Src, pstParam->status_cb.attention_status.attention);
        } else if (op == ESP_BLE_MESH_MODEL_OP_HEALTH_PERIOD_STATUS) {
            printf("\n[HEALTH] Period Status tu 0x%04X: divider=%u\n\n",
                   u16Src, pstParam->status_cb.period_status.fast_period_divisor);
        }
        break;
    }
    case ESP_BLE_MESH_HEALTH_CLIENT_TIMEOUT_EVT:
        LREP_WARNING(TAG, "[HEALTH] Timeout khi gui toi 0x%04X", u16Src);
        break;
    default:
        break;
    }
}

static void app_mesh_LightingCliCb(esp_ble_mesh_light_client_cb_event_t eEvent, 
                                   esp_ble_mesh_light_client_cb_param_t *pstParam)
{
    if (pstParam == NULL || pstParam->params == NULL) return;

    uint16_t u16Src    = pstParam->params->ctx.addr;
    uint16_t u16Dst    = pstParam->params->ctx.recv_dst;
    uint8_t  u8Ttl     = pstParam->params->ctx.recv_ttl;
    int8_t   s8Rssi    = pstParam->params->ctx.recv_rssi;
    uint16_t u16NetIdx = pstParam->params->ctx.net_idx;
    uint16_t u16AppIdx = pstParam->params->ctx.app_idx;
    uint32_t u32Opcode = pstParam->params->opcode;

    if (pstParam->error_code != ESP_OK) {
        LREP_WARNING(TAG, "[LIGHT CLI ERROR] Tu 0x%04X | Event=%d | Opcode=0x%04lX | ErrCode=%d", 
                     u16Src, eEvent, (unsigned long)u32Opcode, pstParam->error_code);
        return;
    }

    switch (eEvent) {
    case ESP_BLE_MESH_LIGHT_CLIENT_GET_STATE_EVT:
    case ESP_BLE_MESH_LIGHT_CLIENT_SET_STATE_EVT:
    case ESP_BLE_MESH_LIGHT_CLIENT_PUBLISH_EVT: {

        /* Cho phép bắt cả Status (0x824E), Get Response (0x824B) và Set Response (0x824C) */
        if (u32Opcode == ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_STATUS || 
            u32Opcode == ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_GET || 
            u32Opcode == ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET ||
            u32Opcode == 0x824E || u32Opcode == 0x824B || u32Opcode == 0x824C) {

            uint16_t u16PresentLightness = pstParam->status_cb.lightness_status.present_lightness;
            bool     bHasTarget          = pstParam->status_cb.lightness_status.op_en;

            printf("\n--------------------------------------------------\n");
            printf("       [BLE MESH LIGHT LIGHTNESS STATUS]          \n");
            printf("--------------------------------------------------\n");
            printf("  Event Type      : %s (%d)\n", 
                   (eEvent == ESP_BLE_MESH_LIGHT_CLIENT_GET_STATE_EVT) ? "GET_RESPONSE" :
                   (eEvent == ESP_BLE_MESH_LIGHT_CLIENT_SET_STATE_EVT) ? "SET_RESPONSE" : "PUBLISH", eEvent);
            printf("  Opcode          : 0x%04lX\n", (unsigned long)u32Opcode);
            printf("  Dia chi Nguon   : 0x%04X\n", u16Src);
            printf("  Dia chi Dich    : 0x%04X (%s)\n", u16Dst, (u16Dst == 0xFFFF) ? "Broadcast" : "Unicast/Group");
            printf("  Thong so Song   : RSSI = %d dBm | TTL = %u\n", s8Rssi, u8Ttl);
            printf("  Key Index       : NetIdx = 0x%04X | AppIdx = 0x%04X\n", u16NetIdx, u16AppIdx);
            printf("  ----------------------------------------------\n");
            printf("  Present Lightness: %u (0x%04X)\n", u16PresentLightness, u16PresentLightness);

            if (bHasTarget) {
                printf("  Target Lightness : %u\n", pstParam->status_cb.lightness_status.target_lightness);
                printf("  Remain Time      : %u ms\n", pstParam->status_cb.lightness_status.remain_time);
            }
            printf("--------------------------------------------------\n\n");
        }
        /* ====================== LIGHT CTL STATUS ====================== */
        if (u32Opcode == ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_STATUS ||
            u32Opcode == 0x8260) {   // LIGHT_CTL_STATUS
            uint16_t present_lightness   = pstParam->status_cb.ctl_status.present_ctl_lightness;
            uint16_t present_temperature = pstParam->status_cb.ctl_status.present_ctl_temperature;
            bool     has_target          = pstParam->status_cb.ctl_status.op_en;

            printf("\n--------------------------------------------------\n");
            printf("       [BLE MESH LIGHT CTL STATUS]                \n");
            printf("--------------------------------------------------\n");
            printf("  Event Type      : %s (%d)\n",
                (eEvent == ESP_BLE_MESH_LIGHT_CLIENT_GET_STATE_EVT) ? "GET_RESPONSE" :
                (eEvent == ESP_BLE_MESH_LIGHT_CLIENT_SET_STATE_EVT) ? "SET_RESPONSE" : "PUBLISH", eEvent);
            printf("  Opcode          : 0x%04lX\n", (unsigned long)u32Opcode);
            printf("  Dia chi Nguon   : 0x%04X\n", u16Src);
            printf("  Dia chi Dich    : 0x%04X\n", u16Dst);
            printf("  RSSI / TTL      : %d dBm / %u\n", s8Rssi, u8Ttl);
            printf("  ----------------------------------------------\n");
            printf("  Present Lightness   : %u\n", present_lightness);
            printf("  Present Temperature : %u (0x%04X)\n", present_temperature, present_temperature);

            if (has_target) {
                printf("  Target Lightness    : %u\n", pstParam->status_cb.ctl_status.target_ctl_lightness);
                printf("  Target Temperature  : %u\n", pstParam->status_cb.ctl_status.target_ctl_temperature);
                printf("  Remain Time         : %u\n", pstParam->status_cb.ctl_status.remain_time);
            }
            printf("--------------------------------------------------\n\n");
        }

        /* ====================== LIGHT CTL TEMPERATURE STATUS ====================== */
        else if (u32Opcode == ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_TEMPERATURE_STATUS ||
                u32Opcode == 0x8266) {
            uint16_t present_temp = pstParam->status_cb.ctl_temperature_status.present_ctl_temperature;
            uint16_t present_delta = pstParam->status_cb.ctl_temperature_status.present_ctl_delta_uv;
            bool     has_target   = pstParam->status_cb.ctl_temperature_status.op_en;

            printf("\n--------------------------------------------------\n");
            printf("   [BLE MESH LIGHT CTL TEMPERATURE STATUS]        \n");
            printf("--------------------------------------------------\n");
            printf("  Event Type      : %s\n",
                (eEvent == ESP_BLE_MESH_LIGHT_CLIENT_PUBLISH_EVT) ? "PUBLISH" : "RESPONSE");
            printf("  Opcode          : 0x%04lX\n", (unsigned long)u32Opcode);
            printf("  Dia chi Nguon   : 0x%04X\n", u16Src);
            printf("  Present Temp    : %u\n", present_temp);
            printf("  Present Delta UV: %d\n", (int16_t)present_delta);
            if (has_target) {
                printf("  Target Temp     : %u\n", pstParam->status_cb.ctl_temperature_status.target_ctl_temperature);
                printf("  Target Delta UV : %d\n", (int16_t)pstParam->status_cb.ctl_temperature_status.target_ctl_delta_uv);
            }
            printf("--------------------------------------------------\n\n");
        }
        break;
    }
    default:
        break;
    }
}

static void app_mesh_CfgCliCb(esp_ble_mesh_cfg_client_cb_event_t eEvent, esp_ble_mesh_cfg_client_cb_param_t *pstParam)
{
}

static void app_mesh_ProvCb(esp_ble_mesh_prov_cb_event_t eEvent, esp_ble_mesh_prov_cb_param_t *pstParam)
{
    st_mesh_evt_t stEvt = { .s64TimeUs = esp_timer_get_time() };

    switch (eEvent) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        LREP(TAG, "Provisioning stack register xong (err=%d)", pstParam->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        LREP(TAG, "[GW] Node prov enable xong (err=%d)", pstParam->node_prov_enable_comp.err_code);
        stEvt.eType = MESH_EVT_ENABLED;
        app_mesh_PostEvt(&stEvt);
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        LREP(TAG, "[GW] DA DUOC CAP MANG: NetIdx=0x%04X, Unicast=0x%04X",
             pstParam->node_prov_complete.net_idx, pstParam->node_prov_complete.addr);
        stEvt.eType    = MESH_EVT_PROV_COMPLETE;
        stEvt.u16Src   = pstParam->node_prov_complete.net_idx;
        stEvt.u16Param = pstParam->node_prov_complete.addr;
        app_mesh_PostEvt(&stEvt);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        LREP_WARNING(TAG, "[GW] Nhan su kien Reset provisioning");
        stEvt.eType = MESH_EVT_PROV_RESET;
        app_mesh_PostEvt(&stEvt);
        break;
    case ESP_BLE_MESH_NODE_ADD_LOCAL_APP_KEY_COMP_EVT:
        stEvt.eType = MESH_EVT_LOCAL_APPKEY_ADD;
        stEvt.u16Src = pstParam->node_add_app_key_comp.net_idx;
        stEvt.u16Param = pstParam->node_add_app_key_comp.app_idx;
        stEvt.iErrCode = pstParam->node_add_app_key_comp.err_code;
        app_mesh_PostEvt(&stEvt);
        break;
    case ESP_BLE_MESH_NODE_BIND_APP_KEY_TO_MODEL_COMP_EVT:
        stEvt.eType = MESH_EVT_LOCAL_APP_BIND;
        stEvt.u16Src = pstParam->node_bind_app_key_to_model_comp.element_addr;
        stEvt.u16Dst = pstParam->node_bind_app_key_to_model_comp.company_id;
        stEvt.u16Param = pstParam->node_bind_app_key_to_model_comp.app_idx;
        stEvt.u16ModelId = pstParam->node_bind_app_key_to_model_comp.model_id;
        stEvt.iErrCode = pstParam->node_bind_app_key_to_model_comp.err_code;
        app_mesh_PostEvt(&stEvt);
        break;
    case ESP_BLE_MESH_MODEL_SUBSCRIBE_GROUP_ADDR_COMP_EVT:
        stEvt.eType = MESH_EVT_LOCAL_SUB_ADD;
        stEvt.u16Src = pstParam->model_sub_group_addr_comp.element_addr;
        stEvt.u16Dst = pstParam->model_sub_group_addr_comp.company_id;
        stEvt.u16ModelId = pstParam->model_sub_group_addr_comp.model_id;
        stEvt.u16Param = pstParam->model_sub_group_addr_comp.group_addr;
        stEvt.iErrCode = pstParam->model_sub_group_addr_comp.err_code;
        app_mesh_PostEvt(&stEvt);
        break;
    default:
        break;
    }
}

static void app_mesh_CfgSrvCb(esp_ble_mesh_cfg_server_cb_event_t eEvent, esp_ble_mesh_cfg_server_cb_param_t *pstParam)
{
    if (eEvent != ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) return;

    st_mesh_evt_t stEvt = { .s64TimeUs = esp_timer_get_time() };

    switch (pstParam->ctx.recv_op) {
    case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
        stEvt.eType    = MESH_EVT_APPKEY_ADD;
        stEvt.u16Src   = pstParam->value.state_change.appkey_add.net_idx;
        stEvt.u16Param = pstParam->value.state_change.appkey_add.app_idx;
        app_mesh_PostEvt(&stEvt);
        break;
    case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
        stEvt.eType    = MESH_EVT_APP_BIND;
        stEvt.u16Param = pstParam->value.state_change.mod_app_bind.app_idx;
        app_mesh_PostEvt(&stEvt);
        break;
    case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
        stEvt.eType    = MESH_EVT_SUB_ADD;
        stEvt.u16Param = pstParam->value.state_change.mod_sub_add.sub_addr;
        app_mesh_PostEvt(&stEvt);
        break;
    default:
        break;
    }
}

static void app_mesh_GenericSrvCb(esp_ble_mesh_generic_server_cb_event_t eEvent, esp_ble_mesh_generic_server_cb_param_t *pstParam)
{
    if (pstParam == NULL) return;

    uint32_t u32Op = pstParam->ctx.recv_op;
    uint16_t u16Src = pstParam->ctx.addr;
    uint16_t u16Dst = pstParam->ctx.recv_dst;

    if (eEvent == ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT) {
        uint8_t u8TargetOnOff = pstParam->value.state_change.onoff_set.onoff;
        LREP(TAG, "[RX] 0x%04X -> 0x%04X | op=0x%04lX | OnOff=%u", u16Src, u16Dst, (unsigned long)u32Op, u8TargetOnOff);

        st_mesh_evt_t stEvt = {
            .eType     = MESH_EVT_ONOFF_SET,
            .s64TimeUs = esp_timer_get_time(),
            .u16Src    = u16Src,
            .u16Dst    = u16Dst,
            .u8OnOff   = u8TargetOnOff,
            .u8Ttl     = pstParam->ctx.recv_ttl,
            .s8Rssi    = pstParam->ctx.recv_rssi,
        };
        app_mesh_PostEvt(&stEvt);
    }
}

/* DUY NHẤT 1 HÀM GENERIC CLIENT CALLBACK */
static void app_mesh_GenericCliCb(esp_ble_mesh_generic_client_cb_event_t eEvent,
                                  esp_ble_mesh_generic_client_cb_param_t *pstParam)
{
    if (pstParam == NULL || pstParam->params == NULL) {
        return;
    }

    uint16_t u16Src    = pstParam->params->ctx.addr;
    uint16_t u16Dst    = pstParam->params->ctx.recv_dst;
    uint8_t  u8Ttl     = pstParam->params->ctx.recv_ttl;
    int8_t   s8Rssi    = pstParam->params->ctx.recv_rssi;
    uint16_t u16NetIdx = pstParam->params->ctx.net_idx;
    uint16_t u16AppIdx = pstParam->params->ctx.app_idx;
    uint32_t u32Opcode = pstParam->params->opcode;

    if (pstParam->error_code != ESP_OK) {
        LREP_WARNING(TAG, "[GENERIC CLI ERROR] Tu 0x%04X | Event=%d | Opcode=0x%04lX | ErrCode=%d",
                     u16Src, eEvent, (unsigned long)u32Opcode, pstParam->error_code);
        return;
    }

    switch (eEvent) {
    case ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT:
    case ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT:
    case ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT:
    {
        /* ====================== GENERIC ONOFF STATUS (0x8204) ====================== */
        if (u32Opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS) {
            uint8_t present   = pstParam->status_cb.onoff_status.present_onoff;
            bool    has_target = pstParam->status_cb.onoff_status.op_en;

            printf("\n--------------------------------------------------\n");
            printf("       [BLE MESH GENERIC ONOFF STATUS]            \n");
            printf("--------------------------------------------------\n");
            printf("  Event Type      : %s (%d)\n",
                   (eEvent == ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT) ? "GET_RESPONSE" :
                   (eEvent == ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT) ? "SET_RESPONSE" : "PUBLISH",
                   eEvent);
            printf("  Opcode          : 0x%04lX\n", (unsigned long)u32Opcode);
            printf("  Dia chi Nguon   : 0x%04X\n", u16Src);
            printf("  Dia chi Dich    : 0x%04X (%s)\n", u16Dst,
                   (u16Dst == 0xFFFF) ? "Broadcast" : "Unicast/Group");
            printf("  Thong so Song   : RSSI = %d dBm | TTL = %u\n", s8Rssi, u8Ttl);
            printf("  Key Index       : NetIdx = 0x%04X | AppIdx = 0x%04X\n", u16NetIdx, u16AppIdx);
            printf("  ----------------------------------------------\n");
            printf("  Present OnOff   : %u (%s)\n", present, present ? "ON" : "OFF");

            if (has_target) {
                printf("  Target OnOff    : %u\n", pstParam->status_cb.onoff_status.target_onoff);
                printf("  Remain Time     : %u\n", pstParam->status_cb.onoff_status.remain_time);
            }
            printf("--------------------------------------------------\n\n");
        }
        /* ====================== GENERIC LEVEL STATUS (0x8208) ====================== */
        else if (u32Opcode == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS) {
            int16_t present_level = pstParam->status_cb.level_status.present_level;
            bool    has_target    = pstParam->status_cb.level_status.op_en;

            printf("\n--------------------------------------------------\n");
            printf("       [BLE MESH GENERIC LEVEL STATUS]            \n");
            printf("--------------------------------------------------\n");
            printf("  Event Type      : %s (%d)\n",
                   (eEvent == ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT) ? "GET_RESPONSE" :
                   (eEvent == ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT) ? "SET_RESPONSE" : "PUBLISH",
                   eEvent);
            printf("  Opcode          : 0x%04lX (GENERIC_LEVEL_STATUS)\n", (unsigned long)u32Opcode);
            printf("  Dia chi Nguon   : 0x%04X\n", u16Src);
            printf("  Dia chi Dich    : 0x%04X (%s)\n", u16Dst,
                   (u16Dst == 0xFFFF) ? "Broadcast" : "Unicast/Group");
            printf("  Thong so Song   : RSSI = %d dBm | TTL = %u\n", s8Rssi, u8Ttl);
            printf("  Key Index       : NetIdx = 0x%04X | AppIdx = 0x%04X\n", u16NetIdx, u16AppIdx);
            printf("  ----------------------------------------------\n");
            printf("  Present Level   : %d (0x%04X)\n", present_level, (uint16_t)present_level);

            if (has_target) {
                printf("  Target Level    : %d\n", pstParam->status_cb.level_status.target_level);
                printf("  Remain Time     : %u\n", pstParam->status_cb.level_status.remain_time);
            }
            printf("--------------------------------------------------\n\n");
        }
        break;
    }

    case ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT:
        LREP_WARNING(TAG, "[GENERIC CLI] Timeout khi gui toi 0x%04X | Opcode=0x%04lX",
                     u16Src, (unsigned long)u32Opcode);
        break;

    default:
        break;
    }
}
#define MESH_DUMP_MAX_PARAMS   64

static uint16_t app_mesh_OpToBytes(uint32_t u32Op, uint8_t *pu8Out)
{
    if (u32Op >= 0xC00000) {
        pu8Out[0] = (uint8_t)(u32Op >> 16);
        pu8Out[1] = (uint8_t)(u32Op & 0xFF);
        pu8Out[2] = (uint8_t)((u32Op >> 8) & 0xFF);
        return 3;
    }
    if (u32Op >= 0x8000) {
        pu8Out[0] = (uint8_t)(u32Op >> 8);
        pu8Out[1] = (uint8_t)(u32Op & 0xFF);
        return 2;
    }
    pu8Out[0] = (uint8_t)u32Op;
    return 1;
}

static void app_mesh_PrintFullMsg(const esp_ble_mesh_msg_ctx_t *pstCtx, uint32_t u32Op, const uint8_t *pu8Params, uint16_t u16ParamLen)
{
    uint8_t  acBytes[3 + MESH_DUMP_MAX_PARAMS];
    uint16_t u16OpLen = app_mesh_OpToBytes(u32Op, acBytes);
    uint16_t u16Copy  = (u16ParamLen < MESH_DUMP_MAX_PARAMS) ? u16ParamLen : MESH_DUMP_MAX_PARAMS;
    uint16_t u16Total = u16OpLen;

    if (pu8Params != NULL && u16Copy > 0) {
        memcpy(&acBytes[u16OpLen], pu8Params, u16Copy);
        u16Total += u16Copy;
    }

    char  acHex[(3 + MESH_DUMP_MAX_PARAMS) * 3 + 1];
    char *p = acHex;
    for (uint16_t i = 0; i < u16Total; i++) {
        p += snprintf(p, sizeof(acHex) - (size_t)(p - acHex), i ? " %02X" : "%02X", acBytes[i]);
    }

    LREP(TAG, "[RX] 0x%04X -> 0x%04X | ttl=%u rssi=%d | len=%u | %s%s",
         pstCtx->addr, pstCtx->recv_dst, pstCtx->recv_ttl, pstCtx->recv_rssi,
         (unsigned)(u16OpLen + u16ParamLen), acHex, (u16ParamLen > u16Copy) ? " ..." : "");
}

static void app_mesh_VendorServerCb(esp_ble_mesh_model_cb_event_t eEvent, esp_ble_mesh_model_cb_param_t *pstParam)
{
    if (!pstParam) return;

    if (eEvent == ESP_BLE_MESH_MODEL_OPERATION_EVT) {
        uint32_t u32Opcode = pstParam->model_operation.opcode;
        uint16_t u16Src    = pstParam->model_operation.ctx->addr;
        uint16_t u16Dst    = pstParam->model_operation.ctx->recv_dst;
        uint16_t u16Length = pstParam->model_operation.length;
        uint8_t *pu8Msg    = pstParam->model_operation.msg;

        app_mesh_PrintFullMsg(pstParam->model_operation.ctx, u32Opcode, pu8Msg, u16Length);
        app_mesh_AppendLog(u16Src, u16Dst, (uint8_t)u32Opcode, 0,
                           pstParam->model_operation.ctx->recv_ttl, pstParam->model_operation.ctx->recv_rssi);

        APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
        if (stGuard.bLocked) {
            g_stInfo.u16LastSrc = u16Src;
            g_stInfo.u16LastDst = u16Dst;
            g_stInfo.u32RxCount++;
        }
    }
}

static bool app_mesh_ModelHasAppKey(const esp_ble_mesh_model_t *pstModel, uint16_t u16AppIdx)
{
    for (size_t i = 0; i < ARRAY_SIZE(pstModel->keys); i++) {
        if (pstModel->keys[i] == u16AppIdx) return true;
    }
    return false;
}

static void app_mesh_UpdateGatewayReady(void)
{
    g_bGatewayJoined = g_bGatewayConfigStarted &&
                       g_u8AppKeyReadyMask == 0x07 &&
                       g_u8BindPendingCount == 0 &&
                       g_u8BindFailedCount == 0 &&
                       g_u8GroupPendingCount == 0 &&
                       !g_bGroupConfigFailed;
    if (g_bGatewayConfigStarted && (g_u8BindFailedCount != 0 || g_bGroupConfigFailed)) {
        g_bGatewayConfigStarted = false;
        g_bGatewayJoined = false;
    }
    if (g_bGatewayJoined) {
        LREP(TAG, "[GW] 3 AppKey da bind; cac model da subscribe Group 0x%04X.", g_u16ConfiguredGroup);
    }
}

static void app_mesh_StartModelBindings(void)
{
    const uint16_t u16AppIdx[] = {
        DF_MESH_HOMI_APP_KEY_IDX_0,
        DF_MESH_HOMI_APP_KEY_IDX_1,
        DF_MESH_HOMI_APP_KEY_IDX_2,
    };

    typedef struct {
        esp_ble_mesh_model_t *pstModel;
        uint16_t u16CompanyId;
        uint16_t u16ModelId;
    } st_model_bind_target_t;

    st_model_bind_target_t astTargets[] = {
        { .pstModel = &g_astRootModels[2], .u16CompanyId = ESP_BLE_MESH_CID_NVAL, .u16ModelId = ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV },
        { .pstModel = &g_astRootModels[3], .u16CompanyId = ESP_BLE_MESH_CID_NVAL, .u16ModelId = ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI },
        { .pstModel = &g_astRootModels[4], .u16CompanyId = ESP_BLE_MESH_CID_NVAL, .u16ModelId = ESP_BLE_MESH_MODEL_ID_HEALTH_CLI },  
        { .pstModel = &g_astRootModels[5], .u16CompanyId = ESP_BLE_MESH_CID_NVAL, .u16ModelId = ESP_BLE_MESH_MODEL_ID_GEN_LEVEL_CLI },
        { .pstModel = &g_astRootModels[6], .u16CompanyId = ESP_BLE_MESH_CID_NVAL, .u16ModelId = ESP_BLE_MESH_MODEL_ID_LIGHT_LIGHTNESS_CLI },
        { .pstModel = &g_astVndModelsServer[0], .u16CompanyId = DF_MESH_VND_COMPANY_ID, .u16ModelId = DF_MESH_VND_MODEL_ID_SERVER },
    };

    g_u8BindPendingCount = 0;
    g_u8BindFailedCount = 0;

    for (size_t key = 0; key < ARRAY_SIZE(u16AppIdx); key++) {
        for (size_t model = 0; model < ARRAY_SIZE(astTargets); model++) {
            esp_ble_mesh_model_t *pstModel = astTargets[model].pstModel;
            uint16_t u16Cid = astTargets[model].u16CompanyId;
            uint16_t u16Mid = astTargets[model].u16ModelId;

            if (app_mesh_ModelHasAppKey(pstModel, u16AppIdx[key])) continue;

            esp_err_t err = esp_ble_mesh_node_bind_app_key_to_local_model(
                esp_ble_mesh_get_primary_element_address(), u16Cid, u16Mid, u16AppIdx[key]);

            if (err == ESP_OK) {
                g_u8BindPendingCount++;
            } else {
                g_u8BindFailedCount++;
                LREP_ERROR(TAG, "Queue bind AppKey 0x%03X vao model 0x%04X (CID 0x%04X) that bai: %s",
                           u16AppIdx[key], pstModel->model_id, u16Cid, esp_err_to_name(err));
            }
        }
    }

    if (g_u8BindPendingCount == 0 && g_u8BindFailedCount == 0) {
        APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
        if (stGuard.bLocked) g_stInfo.bAppBound = true;
    }
}

static void app_mesh_StartGatewayModelConfig(void)
{
    if (!g_bGatewayConfigStarted || g_u8AppKeyReadyMask != 0x07) return;
    app_mesh_StartModelBindings();
    if (app_mesh_SubscribeGroup(g_u16ConfiguredGroup, false) != ERROR_CODE_OK) {
        g_bGroupConfigFailed = true;
    }
    app_mesh_UpdateGatewayReady();
}

static void app_mesh_LoadGatewayConfig(void)
{
    nvs_handle_t hNvs;
    esp_err_t err = nvs_open("meshgw", NVS_READONLY, &hNvs);
    if (err != ESP_OK) return;

    uint16_t u16Value = 0;
    err = nvs_get_u16(hNvs, "group", &u16Value);
    if (err == ESP_OK && u16Value >= 0xC000 && u16Value <= 0xFEFF) {
        g_u16ConfiguredGroup = u16Value;
        g_stInfo.u16Group = u16Value;
    }

    err = nvs_get_u16(hNvs, "netidx", &u16Value);
    if (err == ESP_OK && u16Value <= 0x0FFF) {
        g_u16ProvisionedNetIdx = u16Value;
    }
    nvs_close(hNvs);
}

static int32_t app_mesh_GatewayJoinNetwork(uint16_t u16NetIdx)
{
    if (!g_bMeshInitialized || !esp_ble_mesh_node_is_provisioned()) return ERROR_CODE_FAIL;
    if (u16NetIdx > 0x0FFF || esp_ble_mesh_node_get_local_net_key(u16NetIdx) == NULL) return ERROR_CODE_NOT_FOUND;
    if (g_bGatewayConfigStarted) return g_bGatewayJoined ? ERROR_CODE_OK : ERROR_CODE_BUSY;

    const uint8_t *pKeys[] = { HOMI_APP_KEY_0, HOMI_APP_KEY_1, HOMI_APP_KEY_2 };
    const uint16_t u16AppIdx[] = { DF_MESH_HOMI_APP_KEY_IDX_0, DF_MESH_HOMI_APP_KEY_IDX_1, DF_MESH_HOMI_APP_KEY_IDX_2 };
    
    g_bGatewayConfigStarted = true;
    g_bGatewayJoined = false;
    g_u8AppKeyReadyMask = 0;
    g_u8BindFailedCount = 0;
    g_u8GroupPendingCount = 0;
    g_bGroupConfigFailed = false;

    for (size_t i = 0; i < ARRAY_SIZE(u16AppIdx); i++) {
        const uint8_t *pExistingKey = esp_ble_mesh_node_get_local_app_key(u16AppIdx[i]);
        if (pExistingKey != NULL) {
            if (memcmp(pExistingKey, pKeys[i], 16) != 0) {
                g_bGatewayConfigStarted = false;
                return ERROR_CODE_INVALID_PARAM;
            }
            g_u8AppKeyReadyMask |= (uint8_t)(1U << i);
            continue;
        }

        esp_err_t err = esp_ble_mesh_node_add_local_app_key(pKeys[i], u16NetIdx, u16AppIdx[i]);
        if (err != ESP_OK) {
            g_bGatewayConfigStarted = false;
            return ERROR_CODE_FAIL;
        }
    }

    {
        APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
        if (stGuard.bLocked) {
            g_stInfo.bProvisioned = true;
            g_stInfo.u16Unicast = esp_ble_mesh_get_primary_element_address();
            g_stInfo.u16NetIdx = u16NetIdx;
            g_stInfo.u16AppIdx = DF_MESH_HOMI_APP_KEY_IDX_0;
        }
    }
    g_u16ProvisionedNetIdx = u16NetIdx;

    if (g_u8AppKeyReadyMask == 0x07) app_mesh_StartGatewayModelConfig();
    return ERROR_CODE_OK;
}

static void app_mesh_Task(void *pvParameters)
{
    st_mesh_evt_t stEvt;

    while (1) {
        if (xQueueReceive(g_hMeshEvtQ, &stEvt, portMAX_DELAY) == pdTRUE) {
            switch (stEvt.eType) {
            case MESH_EVT_ENABLED: {
                bool bProv = esp_ble_mesh_node_is_provisioned();
                uint16_t u16Addr = bProv ? esp_ble_mesh_get_primary_element_address() : 0;
                {
                    APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
                    if (stGuard.bLocked) {
                        g_stInfo.bProvisioned = bProv;
                        g_stInfo.u16Unicast   = u16Addr;
                    }
                }
                if (bProv) {
                    g_stInfo.u16NetIdx = g_u16ProvisionedNetIdx;
                    app_mesh_GatewayJoinNetwork(g_u16ProvisionedNetIdx);
                }
                break;
            }
            case MESH_EVT_PROV_COMPLETE: {
                {
                    APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
                    if (stGuard.bLocked) {
                        g_stInfo.bProvisioned = true;
                        g_stInfo.u16NetIdx    = stEvt.u16Src;
                        g_stInfo.u16Unicast   = stEvt.u16Param;
                    }
                }
                g_u16ProvisionedNetIdx = stEvt.u16Src;
                nvs_handle_t hNvs;
                if (nvs_open("meshgw", NVS_READWRITE, &hNvs) == ESP_OK) {
                    nvs_set_u16(hNvs, "netidx", stEvt.u16Src);
                    nvs_commit(hNvs);
                    nvs_close(hNvs);
                }
                app_mesh_GatewayJoinNetwork(stEvt.u16Src);
                break;
            }
            case MESH_EVT_PROV_RESET:
                app_common_DelayMs(300);
                esp_restart();
                break;
            case MESH_EVT_LOCAL_APPKEY_ADD: {
                int iKeyPos = -1;
                if (stEvt.u16Param == DF_MESH_HOMI_APP_KEY_IDX_0) iKeyPos = 0;
                if (stEvt.u16Param == DF_MESH_HOMI_APP_KEY_IDX_1) iKeyPos = 1;
                if (stEvt.u16Param == DF_MESH_HOMI_APP_KEY_IDX_2) iKeyPos = 2;
                if (stEvt.iErrCode != ESP_OK || iKeyPos < 0) {
                    g_bGatewayConfigStarted = false;
                    g_bGatewayJoined = false;
                    break;
                }
                g_u8AppKeyReadyMask |= (uint8_t)(1U << iKeyPos);
                app_mesh_StartGatewayModelConfig();
                break;
            }
            case MESH_EVT_LOCAL_APP_BIND:
                if (stEvt.u16Src != esp_ble_mesh_get_primary_element_address()) break;
                if (g_u8BindPendingCount > 0) g_u8BindPendingCount--;
                if (stEvt.iErrCode != ESP_OK) g_u8BindFailedCount++;
                
                if (g_u8BindPendingCount == 0) {
                    APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
                    if (stGuard.bLocked) g_stInfo.bAppBound = (g_u8BindFailedCount == 0);
                }
                app_mesh_UpdateGatewayReady();
                break;
            case MESH_EVT_LOCAL_SUB_ADD:
                if (stEvt.u16Src != esp_ble_mesh_get_primary_element_address()) break;
                if (g_u8GroupPendingCount > 0) g_u8GroupPendingCount--;
                if (stEvt.iErrCode != ESP_OK) g_bGroupConfigFailed = true;
                if (g_u8GroupPendingCount == 0 && !g_bGroupConfigFailed) {
                    APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
                    if (stGuard.bLocked) g_stInfo.u16Group = g_u16ConfiguredGroup;
                }
                app_mesh_UpdateGatewayReady();
                break;
            case MESH_EVT_ONOFF_SET:
                {
                    APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
                    if (stGuard.bLocked) {
                        g_stInfo.u8AlarmState = stEvt.u8OnOff;
                        g_stInfo.u16LastSrc   = stEvt.u16Src;
                        g_stInfo.u16LastDst   = stEvt.u16Dst;
                        g_stInfo.u32RxCount++;
                    }
                }
                g_stOnoffSrv.state.onoff = stEvt.u8OnOff;
                app_mesh_AppendLog(stEvt.u16Src, stEvt.u16Dst, 0x01, stEvt.u8OnOff, stEvt.u8Ttl, stEvt.s8Rssi);
                break;
            default:
                break;
            }
        }
    }
}

static esp_err_t app_mesh_BluetoothInit(void)
{
#if !CONFIG_IDF_TARGET_ESP32
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
#endif
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    return ESP_OK;
}

int32_t app_mesh_Init(void)
{
    if (g_bMeshInitialized) return ERROR_CODE_OK;

    g_hMeshMutex = xSemaphoreCreateMutex();
    if (!g_hMeshMutex) return ERROR_CODE_NO_MEMORY;

    g_hMeshEvtQ = xQueueCreate(DF_MESH_EVT_QUEUE_LEN, sizeof(st_mesh_evt_t));
    if (!g_hMeshEvtQ) return ERROR_CODE_NO_MEMORY;

    memset(&g_stInfo, 0, sizeof(g_stInfo));
    g_stInfo.u32BootCount = 1;
    g_stInfo.u16NetIdx = g_u16ProvisionedNetIdx;
    g_stInfo.u16AppIdx = DF_MESH_HOMI_APP_KEY_IDX_0;
    app_mesh_LoadGatewayConfig();

    if (xTaskCreate(app_mesh_Task, "fire_mesh_task", DF_MESH_TASK_STACK, NULL, DF_MESH_TASK_PRIO, &g_hMeshTask) != pdPASS) {
        return ERROR_CODE_FAIL;
    }

    if (app_mesh_BluetoothInit() != ESP_OK) return ERROR_CODE_FAIL;

    uint8_t au8Mac[6] = {0};
    esp_read_mac(au8Mac, ESP_MAC_BT);
    memcpy(&g_au8DevUuid[2], au8Mac, sizeof(au8Mac));

    char acName[32];
    snprintf(acName, sizeof(acName), "%s%02X%02X", DF_MESH_DEVICE_NAME_PREFIX, au8Mac[4], au8Mac[5]);

    g_hScanMutex = xSemaphoreCreateMutex();

    esp_ble_mesh_register_prov_callback(app_mesh_ProvCb);
    esp_ble_mesh_register_config_client_callback(app_mesh_CfgCliCb);
    esp_ble_mesh_register_config_server_callback(app_mesh_CfgSrvCb);
    esp_ble_mesh_register_generic_server_callback(app_mesh_GenericSrvCb);
    esp_ble_mesh_register_generic_client_callback(app_mesh_GenericCliCb);
    esp_ble_mesh_register_custom_model_callback(app_mesh_VendorServerCb);
    esp_ble_mesh_register_health_client_callback(app_mesh_HealthCliCb);
    esp_ble_mesh_register_light_client_callback(app_mesh_LightingCliCb);

    esp_err_t err = esp_ble_mesh_init(&g_stProv, &g_stComp);
    if (err != ESP_OK) return ERROR_CODE_FAIL;

    esp_ble_mesh_set_unprovisioned_device_name(acName);
    g_bMeshInitialized = true;

    err = esp_ble_mesh_node_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) {
        g_bMeshInitialized = false;
        return ERROR_CODE_FAIL;
    }

    xTaskCreate(app_mesh_ScannerTask, "mesh_gw_task", 4096, NULL, 3, &g_hScanPrintTask);
    return ERROR_CODE_OK;
}

void app_mesh_PrintStatus(void)
{
    if (!g_bMeshInitialized) return;
    bool bProv = esp_ble_mesh_node_is_provisioned();
    uint16_t u16Addr = bProv ? esp_ble_mesh_get_primary_element_address() : 0;

    st_mesh_info_t stSnap;
    {
        APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
        if (!stGuard.bLocked) return;
        stSnap = g_stInfo;
    }

    printf("\n========================================\n");
    printf("       THONG TIN BLE MESH FIRE NODE     \n");
    printf("========================================\n");
    printf("Trang thai cap mang : %s\n", bProv ? "DA CAP MANG" : "CHUA CAP MANG");
    printf("Unicast Address     : 0x%04X\n", u16Addr);
    printf("NetIdx / AppIdx     : 0x%04X / 0x%04X\n", stSnap.u16NetIdx, stSnap.u16AppIdx);
    printf("========================================\n\n");
}

void app_mesh_PrintLog(void)
{
}

int32_t app_mesh_Reset(void)
{
    if (!g_bMeshInitialized) return ERROR_CODE_FAIL;
    esp_ble_mesh_node_local_reset();
    app_common_DelayMs(500);
    esp_restart();
    return ERROR_CODE_OK;
}

int32_t app_mesh_SendOnOff(uint16_t u16DstAddr, uint8_t u8OnOff)
{
    if (!g_bMeshInitialized) return ERROR_CODE_FAIL;

    esp_ble_mesh_generic_client_set_state_t stSetParam = {
        .onoff_set.onoff = (u8OnOff ? 1 : 0),
        .onoff_set.op_en = false,
    };
    esp_ble_mesh_client_common_param_t stCommonParam = {
        .opcode      = ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK,
        .model       = &g_astRootModels[3],
        .msg_timeout = 0,
        .ctx.net_idx   = g_stInfo.u16NetIdx,
        .ctx.app_idx   = g_stInfo.u16AppIdx,
        .ctx.addr      = u16DstAddr,
        .ctx.send_ttl  = 7,
    };

    esp_err_t err = esp_ble_mesh_generic_client_set_state(&stCommonParam, &stSetParam);
    return (err == ESP_OK) ? ERROR_CODE_OK : ERROR_CODE_FAIL;
}

int32_t app_mesh_SendFireAlarm(uint16_t u16DstAddr, uint8_t u8AlarmState, int16_t s16Temp, uint16_t u16Smoke)
{
    if (!g_bMeshInitialized) return ERROR_CODE_FAIL;

    static uint16_t s_u16PktCounter = 0;
    s_u16PktCounter++;

    st_vendor_custom_data_t stMsg = {
        .u8Cmd          = u8AlarmState ? DF_CMD_FIRE_ALARM_TRIGGER : DF_CMD_FIRE_ALARM_NORMAL,
        .u16PacketId    = s_u16PktCounter,
        .s16Temperature = s16Temp,
        .u16SmokeVal    = u16Smoke,
        .u8AlarmState   = u8AlarmState ? 1 : 0,
    };

    esp_ble_mesh_msg_ctx_t ctx = {
        .net_idx  = g_stInfo.u16NetIdx,
        .app_idx  = g_stInfo.u16AppIdx,
        .addr     = u16DstAddr,
        .send_ttl = 7,
    };

    esp_err_t err = esp_ble_mesh_server_model_send_msg(&g_astVndModelsServer[0], &ctx, DF_MESH_VND_OP_SET, sizeof(stMsg), (uint8_t *)&stMsg);
    return (err == ESP_OK) ? ERROR_CODE_OK : ERROR_CODE_FAIL;
}

int32_t app_mesh_SendVendorRaw(uint16_t u16Dst, uint8_t u8OpIdx, const uint8_t *pu8Data, uint16_t u16Len)
{
    if (!g_bMeshInitialized) return ERROR_CODE_FAIL;
    esp_ble_mesh_msg_ctx_t ctx = {
        .net_idx  = g_stInfo.u16NetIdx,
        .app_idx  = g_stInfo.u16AppIdx,
        .addr     = u16Dst,
        .send_ttl = 7,
    };
    uint32_t u32Op = ESP_BLE_MESH_MODEL_OP_3(u8OpIdx, DF_MESH_VND_COMPANY_ID);
    esp_err_t err = esp_ble_mesh_server_model_send_msg(&g_astVndModelsServer[0], &ctx, u32Op, u16Len, (uint8_t *)pu8Data);
    return (err == ESP_OK) ? ERROR_CODE_OK : ERROR_CODE_FAIL;
}

bool app_mesh_IsProvisioned(void) { return g_bMeshInitialized && esp_ble_mesh_node_is_provisioned(); }
uint8_t app_mesh_GetAlarmState(void) { return g_stInfo.u8AlarmState; }
void app_mesh_SetAlarmState(uint8_t u8AlarmState) { g_stInfo.u8AlarmState = u8AlarmState; }

void app_mesh_PrintScannedDevices(void) {}
void app_mesh_ClearScannedDevices(void) {}
uint32_t app_mesh_GetScannedDeviceCount(void) { return g_u32ScannedCount; }
bool app_mesh_GetGatewayJoined(void) { return g_bGatewayJoined; }

static esp_err_t app_mesh_SubscribeOneModel(esp_ble_mesh_model_t *pstModel, uint16_t u16Group)
{
    if (!pstModel) return ESP_ERR_INVALID_ARG;
    if (esp_ble_mesh_is_model_subscribed_to_group(pstModel, u16Group) != NULL) return ESP_OK;

    uint16_t company_id = (pstModel == &g_astVndModelsServer[0]) ? DF_MESH_VND_COMPANY_ID : ESP_BLE_MESH_CID_NVAL;
    uint16_t model_id   = (pstModel == &g_astVndModelsServer[0]) ? DF_MESH_VND_MODEL_ID_SERVER : pstModel->model_id;

    return esp_ble_mesh_model_subscribe_group_addr(esp_ble_mesh_get_primary_element_address(), company_id, model_id, u16Group);
}

static int32_t app_mesh_SubscribeGroup(uint16_t u16GroupAddr, bool bPersist)
{
    if (!g_bMeshInitialized || !esp_ble_mesh_node_is_provisioned()) return ERROR_CODE_FAIL;

    esp_ble_mesh_model_t *pModels[] = { &g_astRootModels[2], &g_astVndModelsServer[0] };
    g_u8GroupPendingCount = 0;
    g_bGroupConfigFailed = false;

    for (size_t i = 0; i < ARRAY_SIZE(pModels); i++) {
        bool bAlreadySubscribed = (esp_ble_mesh_is_model_subscribed_to_group(pModels[i], u16GroupAddr) != NULL);
        if (app_mesh_SubscribeOneModel(pModels[i], u16GroupAddr) == ESP_OK) {
            if (!bAlreadySubscribed) g_u8GroupPendingCount++;
        } else {
            g_bGroupConfigFailed = true;
            return ERROR_CODE_FAIL;
        }
    }

    if (bPersist) {
        nvs_handle_t hNvs;
        if (nvs_open("meshgw", NVS_READWRITE, &hNvs) == ESP_OK) {
            nvs_set_u16(hNvs, "group", u16GroupAddr);
            nvs_commit(hNvs);
            nvs_close(hNvs);
        }
    }

    g_u16ConfiguredGroup = u16GroupAddr;
    app_mesh_UpdateGatewayReady();
    return ERROR_CODE_OK;
}

int32_t app_mesh_GatewayJoin(void) { return app_mesh_GatewayJoinNetwork(g_u16ProvisionedNetIdx); }
int32_t app_mesh_SetGroupAddress(uint16_t u16GroupAddr) { return app_mesh_SubscribeGroup(u16GroupAddr, true); }
uint16_t app_mesh_GetGroupAddress(void) { return g_u16ConfiguredGroup; }

static void app_mesh_ScannerTask(void *pvParameters)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}

int32_t app_mesh_HealthFaultGet(uint16_t u16Dst, uint16_t u16CompanyId)
{
    if (!g_bMeshInitialized || !esp_ble_mesh_node_is_provisioned()) return ERROR_CODE_FAIL;

    esp_ble_mesh_client_common_param_t common = {
        .opcode       = ESP_BLE_MESH_MODEL_OP_HEALTH_FAULT_GET,
        .model        = &g_astRootModels[4],
        .ctx.net_idx  = g_stInfo.u16NetIdx,
        .ctx.app_idx  = g_stInfo.u16AppIdx,
        .ctx.addr     = u16Dst,
        .ctx.send_ttl = 7,
        .msg_timeout  = 4000,
    };
    esp_ble_mesh_health_client_get_state_t get = { .fault_get.company_id = u16CompanyId };
    return (esp_ble_mesh_health_client_get_state(&common, &get) == ESP_OK) ? ERROR_CODE_OK : ERROR_CODE_FAIL;
}

int32_t app_mesh_HealthAttentionSet(uint16_t u16Dst, uint8_t u8Attention)
{
    if (!g_bMeshInitialized || !esp_ble_mesh_node_is_provisioned()) return ERROR_CODE_FAIL;

    esp_ble_mesh_client_common_param_t common = {
        .opcode       = ESP_BLE_MESH_MODEL_OP_ATTENTION_SET_UNACK,
        .model        = &g_astRootModels[4],
        .ctx.net_idx  = g_stInfo.u16NetIdx,
        .ctx.app_idx  = g_stInfo.u16AppIdx,
        .ctx.addr     = u16Dst,
        .ctx.send_ttl = 7,
        .msg_timeout  = 4000,
    };
    esp_ble_mesh_health_client_set_state_t set = { .attention_set.attention = u8Attention };
    return (esp_ble_mesh_health_client_set_state(&common, &set) == ESP_OK) ? ERROR_CODE_OK : ERROR_CODE_FAIL;
}

int32_t app_mesh_HealthAttentionGet(uint16_t u16Dst)
{
    if (!g_bMeshInitialized || !esp_ble_mesh_node_is_provisioned()) return ERROR_CODE_FAIL;

    esp_ble_mesh_client_common_param_t common = {
        .opcode       = ESP_BLE_MESH_MODEL_OP_ATTENTION_GET,
        .model        = &g_astRootModels[4],
        .ctx.net_idx  = g_stInfo.u16NetIdx,
        .ctx.app_idx  = g_stInfo.u16AppIdx,
        .ctx.addr     = u16Dst,
        .ctx.send_ttl = 7,
        .msg_timeout  = 4000,
    };
    esp_ble_mesh_health_client_get_state_t get = {0};
    return (esp_ble_mesh_health_client_get_state(&common, &get) == ESP_OK) ? ERROR_CODE_OK : ERROR_CODE_FAIL;
}

/* ================== DINH NGHIA HÀM SEND GENERIC LEVEL ================== */
int32_t app_mesh_SendGenericLevelSet(uint16_t u16Dst, int16_t s16Level, bool bAck)
{
    if (!g_bMeshInitialized) return ERROR_CODE_FAIL;

    static uint8_t u8Tid = 0; /* Transaction ID gia tang */

    esp_ble_mesh_generic_client_set_state_t stSet = {
        .level_set.level = s16Level,
        .level_set.tid   = u8Tid++,
        .level_set.op_en = false,
    };

    esp_ble_mesh_client_common_param_t stCommon = {
        .opcode      = bAck ? ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET : ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET_UNACK,
        .model       = &g_astRootModels[5],
        .msg_timeout = 3000,
        .ctx.net_idx = g_stInfo.u16NetIdx,
        .ctx.app_idx = g_stInfo.u16AppIdx,
        .ctx.addr    = u16Dst,
        .ctx.send_ttl = 7,
    };

    esp_err_t err = esp_ble_mesh_generic_client_set_state(&stCommon, &stSet);
    if (err == ESP_OK) {
        LREP(TAG, "[TX LEVEL SET] -> 0x%04X | Level=%d | Tid=%u", u16Dst, s16Level, stSet.level_set.tid);
    } else {
        LREP_ERROR(TAG, "[TX LEVEL SET FAILED] -> 0x%04X | err=0x%x", u16Dst, err);
    }

    return (err == ESP_OK) ? ERROR_CODE_OK : ERROR_CODE_FAIL;
}

int32_t app_mesh_SendGenericLevelGet(uint16_t u16Dst)
{
    if (!g_bMeshInitialized) return ERROR_CODE_FAIL;

    /* Khoi tao struct Get State va gan tham so neu can */
    esp_ble_mesh_generic_client_get_state_t stGet;
    memset(&stGet, 0, sizeof(esp_ble_mesh_generic_client_get_state_t));

    esp_ble_mesh_client_common_param_t stCommon = {
        .opcode      = ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_GET,
        .model       = &g_astRootModels[5], /* Generic Level Client Model */
        .msg_timeout = 3000,
        .ctx.net_idx = g_stInfo.u16NetIdx,
        .ctx.app_idx = g_stInfo.u16AppIdx,
        .ctx.addr    = u16Dst,
        .ctx.send_ttl = 7,
    };

    /* Truyen con tro stGet vao SDK */
    esp_err_t err = esp_ble_mesh_generic_client_get_state(&stCommon, &stGet);
    if (err == ESP_OK) {
        LREP(TAG, "[TX LEVEL GET] -> 0x%04X thanh cong", u16Dst);
    } else {
        LREP_ERROR(TAG, "[TX LEVEL GET FAILED] -> 0x%04X | err=0x%x", u16Dst, err);
    }

    return (err == ESP_OK) ? ERROR_CODE_OK : ERROR_CODE_FAIL;
}



int32_t app_mesh_SendLightnessGet(uint16_t u16Dst)
{
    if (!g_bMeshInitialized) return ERROR_CODE_FAIL;

    esp_ble_mesh_light_client_get_state_t stGet;
    memset(&stGet, 0, sizeof(esp_ble_mesh_light_client_get_state_t));

    esp_ble_mesh_client_common_param_t stCommon = {
        .opcode      = ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_GET, /* 0x824B */
        .model       = &g_astRootModels[6],                         /* Model Index 6 */
        .msg_timeout = 3000,
        .ctx.net_idx = g_stInfo.u16NetIdx,
        .ctx.app_idx = g_stInfo.u16AppIdx,
        .ctx.addr    = u16Dst,
        .ctx.send_ttl = 7,
    };

    esp_err_t err = esp_ble_mesh_light_client_get_state(&stCommon, &stGet);
    if (err == ESP_OK) {
        LREP(TAG, "[TX LIGHTNESS GET] -> 0x%04X thanh cong", u16Dst);
    } else {
        LREP_ERROR(TAG, "[TX LIGHTNESS GET FAILED] -> 0x%04X | err=0x%x", u16Dst, err);
    }

    return (err == ESP_OK) ? ERROR_CODE_OK : ERROR_CODE_FAIL;
}

int32_t app_mesh_SendLightnessSet(uint16_t u16Dst, uint16_t u16Lightness, bool bAck)
{
    if (!g_bMeshInitialized) return ERROR_CODE_FAIL;

    static uint8_t u8Tid = 0;

    esp_ble_mesh_light_client_set_state_t stSet;
    memset(&stSet, 0, sizeof(esp_ble_mesh_light_client_set_state_t));

    /* Gan gia tri cho lightness set */
    stSet.lightness_set.lightness = u16Lightness;
    stSet.lightness_set.tid       = u8Tid++;
    stSet.lightness_set.op_en     = false;

    esp_ble_mesh_client_common_param_t stCommon = {
        .opcode      = bAck ? ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET : ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET_UNACK,
        .model       = &g_astRootModels[6], /* Index 6 */
        .msg_timeout = 3000,
        .ctx.net_idx = g_stInfo.u16NetIdx,
        .ctx.app_idx = g_stInfo.u16AppIdx,
        .ctx.addr    = u16Dst,
        .ctx.send_ttl = 7,
    };

    esp_err_t err = esp_ble_mesh_light_client_set_state(&stCommon, &stSet);
    if (err == ESP_OK) {
        LREP(TAG, "[TX LIGHTNESS SET] -> 0x%04X | Lightness=%u | Tid=%u", u16Dst, u16Lightness, stSet.lightness_set.tid);
    } else {
        LREP_ERROR(TAG, "[TX LIGHTNESS SET FAILED] -> 0x%04X | err=0x%x", u16Dst, err);
    }

    return (err == ESP_OK) ? ERROR_CODE_OK : ERROR_CODE_FAIL;
}

int32_t app_mesh_SendOnOffGet(uint16_t u16Dst)
{
    if (!g_bMeshInitialized) return ERROR_CODE_FAIL;

    esp_ble_mesh_generic_client_get_state_t stGet = {0};
    esp_ble_mesh_client_common_param_t stCommon = {
        .opcode      = ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET,
        .model       = &g_astRootModels[3],   // OnOff Client
        .msg_timeout = 3000,
        .ctx.net_idx = g_stInfo.u16NetIdx,
        .ctx.app_idx = g_stInfo.u16AppIdx,
        .ctx.addr    = u16Dst,
        .ctx.send_ttl = 7,
    };

    esp_err_t err = esp_ble_mesh_generic_client_get_state(&stCommon, &stGet);
    if (err == ESP_OK) {
        LREP(TAG, "[TX ONOFF GET] -> 0x%04X", u16Dst);
    } else {
        LREP_ERROR(TAG, "[TX ONOFF GET FAILED] -> 0x%04X | err=0x%x", u16Dst, err);
    }
    return (err == ESP_OK) ? ERROR_CODE_OK : ERROR_CODE_FAIL;
}

int32_t app_mesh_SendOnOffSet(uint16_t u16Dst, uint8_t u8OnOff, bool bAck)
{
    if (!g_bMeshInitialized) return ERROR_CODE_FAIL;

    static uint8_t u8Tid = 0;

    esp_ble_mesh_generic_client_set_state_t stSet = {
        .onoff_set.onoff = u8OnOff ? 1 : 0,
        .onoff_set.tid   = u8Tid++,
        .onoff_set.op_en = false,
    };

    esp_ble_mesh_client_common_param_t stCommon = {
        .opcode      = bAck ? ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET
                            : ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK,
        .model       = &g_astRootModels[3],
        .msg_timeout = 3000,
        .ctx.net_idx = g_stInfo.u16NetIdx,
        .ctx.app_idx = g_stInfo.u16AppIdx,
        .ctx.addr    = u16Dst,
        .ctx.send_ttl = 7,
    };

    esp_err_t err = esp_ble_mesh_generic_client_set_state(&stCommon, &stSet);
    if (err == ESP_OK) {
        LREP(TAG, "[TX ONOFF SET] -> 0x%04X | OnOff=%u | Ack=%d", u16Dst, u8OnOff, bAck);
    } else {
        LREP_ERROR(TAG, "[TX ONOFF SET FAILED] -> 0x%04X | err=0x%x", u16Dst, err);
    }
    return (err == ESP_OK) ? ERROR_CODE_OK : ERROR_CODE_FAIL;
}


int32_t app_mesh_SendCtlGet(uint16_t u16Dst)
{
    if (!g_bMeshInitialized) return ERROR_CODE_FAIL;

    esp_ble_mesh_light_client_get_state_t stGet;
    memset(&stGet, 0, sizeof(stGet));

    esp_ble_mesh_client_common_param_t stCommon = {
        .opcode      = ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_GET,          // 0x825D
        .model       = &g_astRootModels[7],                          // CTL Client
        .msg_timeout = 3000,
        .ctx.net_idx = g_stInfo.u16NetIdx,
        .ctx.app_idx = g_stInfo.u16AppIdx,
        .ctx.addr    = u16Dst,
        .ctx.send_ttl = 7,
    };

    esp_err_t err = esp_ble_mesh_light_client_get_state(&stCommon, &stGet);
    if (err == ESP_OK)
        LREP(TAG, "[TX CTL GET] -> 0x%04X", u16Dst);
    else
        LREP_ERROR(TAG, "[TX CTL GET FAILED] -> 0x%04X | err=0x%x", u16Dst, err);

    return (err == ESP_OK) ? ERROR_CODE_OK : ERROR_CODE_FAIL;
}

int32_t app_mesh_SendCtlSet(uint16_t u16Dst, uint16_t lightness, uint16_t temperature, int16_t delta_uv, bool bAck)
{
    if (!g_bMeshInitialized) return ERROR_CODE_FAIL;

    static uint8_t u8Tid = 0;

    esp_ble_mesh_light_client_set_state_t stSet;
    memset(&stSet, 0, sizeof(stSet));

    stSet.ctl_set.ctl_lightness   = lightness;
    stSet.ctl_set.ctl_temperature = temperature;
    stSet.ctl_set.ctl_delta_uv    = delta_uv;
    stSet.ctl_set.tid             = u8Tid++;
    stSet.ctl_set.op_en           = false;

    esp_ble_mesh_client_common_param_t stCommon = {
        .opcode      = bAck ? ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_SET
                            : ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_SET_UNACK,
        .model       = &g_astRootModels[7],
        .msg_timeout = 3000,
        .ctx.net_idx = g_stInfo.u16NetIdx,
        .ctx.app_idx = g_stInfo.u16AppIdx,
        .ctx.addr    = u16Dst,
        .ctx.send_ttl = 7,
    };

    esp_err_t err = esp_ble_mesh_light_client_set_state(&stCommon, &stSet);
    if (err == ESP_OK)
        LREP(TAG, "[TX CTL SET] -> 0x%04X | L=%u T=%u ΔUV=%d | Ack=%d",
             u16Dst, lightness, temperature, delta_uv, bAck);
    else
        LREP_ERROR(TAG, "[TX CTL SET FAILED] -> 0x%04X | err=0x%x", u16Dst, err);

    return (err == ESP_OK) ? ERROR_CODE_OK : ERROR_CODE_FAIL;
}

int32_t app_mesh_SendCtlTempGet(uint16_t u16Dst)
{
    if (!g_bMeshInitialized) return ERROR_CODE_FAIL;

    esp_ble_mesh_light_client_get_state_t stGet;
    memset(&stGet, 0, sizeof(stGet));

    esp_ble_mesh_client_common_param_t stCommon = {
        .opcode      = ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_TEMPERATURE_GET, // 0x8261
        .model       = &g_astRootModels[7],
        .msg_timeout = 3000,
        .ctx.net_idx = g_stInfo.u16NetIdx,
        .ctx.app_idx = g_stInfo.u16AppIdx,
        .ctx.addr    = u16Dst,
        .ctx.send_ttl = 7,
    };

    esp_err_t err = esp_ble_mesh_light_client_get_state(&stCommon, &stGet);
    return (err == ESP_OK) ? ERROR_CODE_OK : ERROR_CODE_FAIL;
}

int32_t app_mesh_SendCtlTempSet(uint16_t u16Dst, uint16_t temperature, int16_t delta_uv, bool bAck)
{
    if (!g_bMeshInitialized) return ERROR_CODE_FAIL;

    static uint8_t u8Tid = 0;

    esp_ble_mesh_light_client_set_state_t stSet;
    memset(&stSet, 0, sizeof(stSet));

    stSet.ctl_temperature_set.ctl_temperature = temperature;
    stSet.ctl_temperature_set.ctl_delta_uv    = delta_uv;
    stSet.ctl_temperature_set.tid             = u8Tid++;
    stSet.ctl_temperature_set.op_en           = false;

    esp_ble_mesh_client_common_param_t stCommon = {
        .opcode      = bAck ? ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_TEMPERATURE_SET
                            : ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_TEMPERATURE_SET_UNACK,
        .model       = &g_astRootModels[7],
        .msg_timeout = 3000,
        .ctx.net_idx = g_stInfo.u16NetIdx,
        .ctx.app_idx = g_stInfo.u16AppIdx,
        .ctx.addr    = u16Dst,
        .ctx.send_ttl = 7,
    };

    esp_err_t err = esp_ble_mesh_light_client_set_state(&stCommon, &stSet);
    if (err == ESP_OK)
        LREP(TAG, "[TX CTL TEMP SET] -> 0x%04X | T=%u ΔUV=%d", u16Dst, temperature, delta_uv);
    return (err == ESP_OK) ? ERROR_CODE_OK : ERROR_CODE_FAIL;
}