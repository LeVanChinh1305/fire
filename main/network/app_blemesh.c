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

#include <stdio.h>
#include <string.h>

static const char *TAG = "FIRE_MESH";

#define MESH_INFO_MUTEX_NAME    "FIRE_MESH_MUTEX"
#define DF_LOCK_TIMEOUT_MS      1000

#ifndef ROLE_NODE
#define ROLE_NODE 0
#endif

/* ================== KIá»‚U Dá»® LIá»†U Sá»° KIá»†N ================== */
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

/* Tráº¡ng thÃ¡i Node */
typedef struct {
    bool     bProvisioned;
    uint16_t u16Unicast;
    uint16_t u16NetIdx;
    uint16_t u16AppIdx;
    bool     bAppBound;
    uint16_t u16Group;
    uint8_t  u8AlarmState;  /* 0 = Normal, 1 = Fire Alarm */
    uint32_t u32BootCount;
    uint32_t u32RxCount;
    uint32_t u32TxCount;
    uint16_t u16LastSrc;
    uint16_t u16LastDst;
} st_mesh_info_t;

/* Ring buffer log */
typedef struct {
    int64_t  s64TimeUs;
    uint16_t u16Src;
    uint16_t u16Dst;
    uint8_t  u8Cmd;
    uint8_t  u8State;
    uint8_t  u8Ttl;
    int8_t   s8Rssi;
} st_mesh_log_t;

/* ================== Dá»® LIá»†U VÃ€ TÃ€I NGUYÃŠN ================== */
static st_mesh_info_t    g_stInfo;
static st_mesh_log_t     g_astLog[DF_MESH_LOG_DEPTH];
static uint32_t          g_u32LogHead       = 0;
static uint32_t          g_u32LogCount      = 0;
static SemaphoreHandle_t g_hMeshMutex       = NULL;
static QueueHandle_t     g_hMeshEvtQ        = NULL;
static TaskHandle_t      g_hMeshTask        = NULL;
static bool              g_bMeshInitialized = false;
static uint32_t          g_u32DropEvt       = 0;

/* AppKeys must already exist in the HOMI network and use these indexes. */
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

/* ================== QUáº¢N LÃ QUÃ‰T THIáº¾T Bá»Š BLE MESH ================== */
#define DF_MAX_SCANNED_DEVICES 32

typedef struct {
    uint8_t  au8Mac[6];
    uint8_t  u8AddrType;       /* 0: Public, 1: Random */
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

/* UUID thiáº¿t bá»‹: 0x32, 0x10 + 6 byte MAC */
static uint8_t g_au8DevUuid[16] = { 0x32, 0x10 };

/* ================== KHAI BÃO MODEL BLE MESH ================== */
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

/* Generic OnOff Server & Client */
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stOnoffPub, 2 + 3, ROLE_NODE);

static esp_ble_mesh_gen_onoff_srv_t g_stOnoffSrv = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
};

static esp_ble_mesh_client_t g_stCfgCli;
static esp_ble_mesh_client_t g_stOnoffCli;
static esp_ble_mesh_client_t g_stHealthCli;

static esp_ble_mesh_model_t g_astRootModels[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&g_stCfgSrv),
    ESP_BLE_MESH_MODEL_CFG_CLI(&g_stCfgCli),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&g_stOnoffPub, &g_stOnoffSrv),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(&g_stOnoffPub, &g_stOnoffCli),
    ESP_BLE_MESH_MODEL_HEALTH_CLI(&g_stHealthCli),
};

/* Vendor Server & Client Models */
ESP_BLE_MESH_MODEL_PUB_DEFINE(g_stVndSrvPub, 3 + sizeof(st_vendor_custom_data_t), ROLE_NODE);

/* Wildcard: nhận mọi opcode vendor có Company ID 0x8888 (byte đầu 0xC0..0xFF) */
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

/* ================== TIá»†N ÃCH Ná»˜I Bá»˜ ================== */
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
    if (pstParam == NULL) {
        return;
    }

    uint16_t u16Src = 0;
    uint16_t u16Dst = 0;
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

        if (pstParam->error_code != ESP_OK) {
            break;
        }

        /* Fault Status / Current Status */
        if (op == ESP_BLE_MESH_MODEL_OP_HEALTH_FAULT_STATUS ||
            op == ESP_BLE_MESH_MODEL_OP_HEALTH_CURRENT_STATUS) {
            uint8_t  test_id  = pstParam->status_cb.fault_status.test_id;
            uint16_t company  = pstParam->status_cb.fault_status.company_id;
            struct net_buf_simple *pFaults = pstParam->status_cb.fault_status.fault_array;

            printf("\n[HEALTH] Fault Status tu 0x%04X\n", u16Src);
            printf("  Test ID     : 0x%02X\n", test_id);
            printf("  Company ID  : 0x%04X\n", company);

            if (pFaults && pFaults->len > 0) {
                printf("  So loi      : %u\n", (unsigned)pFaults->len);
                printf("  Ma loi      :");
                for (uint16_t i = 0; i < pFaults->len; i++) {
                    printf(" 0x%02X", pFaults->data[i]);
                }
                printf("\n");
            } else {
                printf("  (Khong co loi)\n");
            }
            printf("\n");
        }
        /* Attention Status – tên opcode đúng IDF 5.x */
        else if (op == ESP_BLE_MESH_MODEL_OP_ATTENTION_STATUS) {
            printf("\n[HEALTH] Attention Status tu 0x%04X: %u giay\n\n",
                   u16Src, pstParam->status_cb.attention_status.attention);
        }
        /* Period Status */
        else if (op == ESP_BLE_MESH_MODEL_OP_HEALTH_PERIOD_STATUS) {
            printf("\n[HEALTH] Period Status tu 0x%04X: divider=%u\n\n",
                   u16Src, pstParam->status_cb.period_status.fast_period_divisor);
        }
        break;
    }

    case ESP_BLE_MESH_HEALTH_CLIENT_TIMEOUT_EVT:
        LREP_WARNING(TAG, "[HEALTH] Timeout khi gui toi 0x%04X", u16Src);
        printf("[HEALTH] Timeout - thiet bi 0x%04X khong tra loi\n", u16Src);
        break;

    default:
        break;
    }
}
/* ================== Xá»¬ LÃ LÆ¯U TRá»® VÃ€ PHÃ‚N TÃCH THIáº¾T Bá»Š BLE MESH QUÃ‰T ÄÆ¯á»¢C ================== */
static void __attribute__((unused)) app_mesh_ScannerOnDeviceFound(const uint8_t au8Mac[6], uint8_t u8AddrType,
                                          const uint8_t au8Uuid[16], bool bHasUuid,
                                          const char *pcDeviceType, int8_t s8Rssi, uint16_t u16OobInfo)
{
    if (g_hScanMutex == NULL || au8Mac == NULL) return;
    if (xSemaphoreTake(g_hScanMutex, pdMS_TO_TICKS(50)) != pdTRUE) return;

    int64_t s64NowUs = esp_timer_get_time();

    /* Tim xem thiet bi da tung xuat hien chua */
    for (uint32_t i = 0; i < g_u32ScannedCount; i++) {
        if (memcmp(g_astScannedDevs[i].au8Mac, au8Mac, 6) == 0) {
            g_astScannedDevs[i].s8Rssi = s8Rssi;
            g_astScannedDevs[i].s64LastSeenUs = s64NowUs;
            g_astScannedDevs[i].u32PktCount++;
            if (bHasUuid && !g_astScannedDevs[i].bHasUuid && au8Uuid != NULL) {
                memcpy(g_astScannedDevs[i].au8Uuid, au8Uuid, 16);
                g_astScannedDevs[i].bHasUuid = true;
            }
            if (pcDeviceType && pcDeviceType[0] != '\0') {
                strncpy(g_astScannedDevs[i].acDeviceType, pcDeviceType, sizeof(g_astScannedDevs[i].acDeviceType) - 1);
            }
            if (u16OobInfo != 0) {
                g_astScannedDevs[i].u16OobInfo = u16OobInfo;
            }
            xSemaphoreGive(g_hScanMutex);
            return;
        }
    }

    /* Neu la thiet bi moi va con cho trong danh sach */
    if (g_u32ScannedCount < DF_MAX_SCANNED_DEVICES) {
        st_mesh_scanned_dev_t *pNew = &g_astScannedDevs[g_u32ScannedCount];
        memset(pNew, 0, sizeof(*pNew));
        memcpy(pNew->au8Mac, au8Mac, 6);
        pNew->u8AddrType = u8AddrType;
        if (bHasUuid && au8Uuid != NULL) {
            memcpy(pNew->au8Uuid, au8Uuid, 16);
            pNew->bHasUuid = true;
        }
        if (pcDeviceType) {
            strncpy(pNew->acDeviceType, pcDeviceType, sizeof(pNew->acDeviceType) - 1);
        }
        pNew->s8Rssi = s8Rssi;
        pNew->u16OobInfo = u16OobInfo;
        pNew->u32PktCount = 1;
        pNew->s64FirstSeenUs = s64NowUs;
        pNew->s64LastSeenUs = s64NowUs;
        g_u32ScannedCount++;
    }

    xSemaphoreGive(g_hScanMutex);
}

/* ==== Coexistence BLE Scan Ä‘Ã£ Ä‘Æ°á»£c táº¯t (khÃ´ng cÃ²n lÃ  Provisioner Scanner) ==== */

static void app_mesh_CfgCliCb(esp_ble_mesh_cfg_client_cb_event_t eEvent, esp_ble_mesh_cfg_client_cb_param_t *pstParam)
{
    /* Che do chi quet: khong can xu ly config client */
}

/* ================== CALLBACK Tá»ª BLE MESH STACK ================== */
static void app_mesh_ProvCb(esp_ble_mesh_prov_cb_event_t eEvent, esp_ble_mesh_prov_cb_param_t *pstParam)
{
    st_mesh_evt_t stEvt = { .s64TimeUs = esp_timer_get_time() };

    switch (eEvent) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        LREP(TAG, "Provisioning stack register xong (err=%d)", pstParam->prov_register_comp.err_code);
        break;

    /* --- Node Events --- */
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        LREP(TAG, "[GW] Node prov enable xong (err=%d) - Gateway san sang lang nghe mang HOMI",
             pstParam->node_prov_enable_comp.err_code);
        stEvt.eType = MESH_EVT_ENABLED;
        app_mesh_PostEvt(&stEvt);
        break;

    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        LREP(TAG, "[GW] Ket noi cap mang MO (Bearer: %s)",
             pstParam->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;

    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        LREP(TAG, "[GW] Ket noi cap mang DONG (Bearer: %s)",
             pstParam->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;

    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        LREP(TAG, "[GW] DA DUOC CAP MANG: NetIdx=0x%04X, Unicast=0x%04X",
             pstParam->node_prov_complete.net_idx,
             pstParam->node_prov_complete.addr);
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

    /* --- Local Key Events (sau khi inject NetKey/AppKey thu cong) --- */
    case ESP_BLE_MESH_NODE_ADD_LOCAL_NET_KEY_COMP_EVT:
        LREP(TAG, "[GW] Them NetKey local hoan tat: net_idx=0x%04X, err=%d",
             pstParam->node_add_net_key_comp.net_idx,
             pstParam->node_add_net_key_comp.err_code);
        break;

    case ESP_BLE_MESH_NODE_ADD_LOCAL_APP_KEY_COMP_EVT:
        LREP(TAG, "[GW] Them AppKey local hoan tat: net_idx=0x%04X, app_idx=0x%04X, err=%d",
             pstParam->node_add_app_key_comp.net_idx,
             pstParam->node_add_app_key_comp.app_idx,
             pstParam->node_add_app_key_comp.err_code);
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

    case ESP_BLE_MESH_MODEL_UNSUBSCRIBE_GROUP_ADDR_COMP_EVT:
        stEvt.eType = MESH_EVT_LOCAL_SUB_DEL;
        stEvt.u16Src = pstParam->model_unsub_group_addr_comp.element_addr;
        stEvt.u16Dst = pstParam->model_unsub_group_addr_comp.company_id;
        stEvt.u16ModelId = pstParam->model_unsub_group_addr_comp.model_id;
        stEvt.u16Param = pstParam->model_unsub_group_addr_comp.group_addr;
        stEvt.iErrCode = pstParam->model_unsub_group_addr_comp.err_code;
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

    case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_DELETE:
        stEvt.eType    = MESH_EVT_SUB_DEL;
        stEvt.u16Param = pstParam->value.state_change.mod_sub_delete.sub_addr;
        app_mesh_PostEvt(&stEvt);
        break;

    default:
        break;
    }
}

static void app_mesh_GenericSrvCb(esp_ble_mesh_generic_server_cb_event_t eEvent, esp_ble_mesh_generic_server_cb_param_t *pstParam)
{
    if (pstParam == NULL) {
        return;
    }

    uint32_t u32Op = pstParam->ctx.recv_op;
    uint16_t u16Src = pstParam->ctx.addr;
    uint16_t u16Dst = pstParam->ctx.recv_dst;

    if (eEvent == ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT) {
        LREP(TAG, "[RX] 0x%04X -> 0x%04X | ttl=%u rssi=%d | op=0x%04lX | GET OnOff",
             u16Src, u16Dst, pstParam->ctx.recv_ttl, pstParam->ctx.recv_rssi,
             (unsigned long)u32Op);
        
        app_mesh_AppendLog(u16Src, u16Dst, (uint8_t)u32Op, g_stOnoffSrv.state.onoff,
                           pstParam->ctx.recv_ttl, pstParam->ctx.recv_rssi);
        return;
    }

    if (eEvent == ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT) {
        uint8_t u8TargetOnOff = pstParam->value.state_change.onoff_set.onoff;

        LREP(TAG, "[RX] 0x%04X -> 0x%04X | ttl=%u rssi=%d | op=0x%04lX | OnOff=%u",
             u16Src, u16Dst, pstParam->ctx.recv_ttl, pstParam->ctx.recv_rssi,
             (unsigned long)u32Op, u8TargetOnOff);

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

static void app_mesh_GenericCliCb(esp_ble_mesh_generic_client_cb_event_t eEvent, esp_ble_mesh_generic_client_cb_param_t *pstParam)
{
    if (pstParam == NULL) {
        return;
    }

    if (pstParam->params != NULL) {
        LREP(TAG, "[RX] 0x%04X -> 0x%04X | ttl=%u rssi=%d | op=0x%04lX | OnOff status=%u (event=%d)",
             pstParam->params->ctx.addr, pstParam->params->ctx.recv_dst,
             pstParam->params->ctx.recv_ttl, pstParam->params->ctx.recv_rssi,
             (unsigned long)pstParam->params->opcode,
             pstParam->status_cb.onoff_status.present_onoff, eEvent);
        if (pstParam->error_code == ESP_OK) {
            app_mesh_AppendLog(pstParam->params->ctx.addr,
                               pstParam->params->ctx.recv_dst,
                               (uint8_t)pstParam->params->ctx.recv_op,
                               pstParam->status_cb.onoff_status.present_onoff,
                               pstParam->params->ctx.recv_ttl,
                               pstParam->params->ctx.recv_rssi);
        }
    }

    if (eEvent == ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT) {
        if (pstParam && pstParam->error_code == ESP_OK) {
            LREP(TAG, "[GEN CLI] Gui lenh OnOff Set thanh cong");
        } else if (pstParam) {
            LREP_WARNING(TAG, "[GEN CLI] Gui lenh OnOff Set that bai: %d", pstParam->error_code);
        }
    }
}

#define MESH_DUMP_MAX_PARAMS   64

/* Doi so opcode ve dung cac byte tren khong: 1, 2 hoac 3 byte */
static uint16_t app_mesh_OpToBytes(uint32_t u32Op, uint8_t *pu8Out)
{
    if (u32Op >= 0xC00000) {            /* vendor: b0, CID thap, CID cao */
        pu8Out[0] = (uint8_t)(u32Op >> 16);
        pu8Out[1] = (uint8_t)(u32Op & 0xFF);
        pu8Out[2] = (uint8_t)((u32Op >> 8) & 0xFF);
        return 3;
    }
    if (u32Op >= 0x8000) {              /* SIG 2 byte */
        pu8Out[0] = (uint8_t)(u32Op >> 8);
        pu8Out[1] = (uint8_t)(u32Op & 0xFF);
        return 2;
    }
    pu8Out[0] = (uint8_t)u32Op;         /* SIG 1 byte */
    return 1;
}

/* In ban tin day du (opcode + tham so) tren MOT dong */
static void app_mesh_PrintFullMsg(const esp_ble_mesh_msg_ctx_t *pstCtx, uint32_t u32Op,
                                  const uint8_t *pu8Params, uint16_t u16ParamLen)
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
         (unsigned)(u16OpLen + u16ParamLen), acHex,
         (u16ParamLen > u16Copy) ? " ..." : "");
}

static void app_mesh_VendorServerCb(esp_ble_mesh_model_cb_event_t eEvent, esp_ble_mesh_model_cb_param_t *pstParam)
{
    if (!pstParam) return;

    switch (eEvent) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT: {
        uint32_t u32Opcode = pstParam->model_operation.opcode;
        uint16_t u16Src    = pstParam->model_operation.ctx->addr;
        uint16_t u16Dst    = pstParam->model_operation.ctx->recv_dst;
        uint16_t u16Length = pstParam->model_operation.length;
        uint8_t *pu8Msg    = pstParam->model_operation.msg;

        /* In toan bo ban tin tren mot dong */
        app_mesh_PrintFullMsg(pstParam->model_operation.ctx, u32Opcode, pu8Msg, u16Length);

        /* Lưu lịch sử vào Ring Buffer */
        app_mesh_AppendLog(u16Src, u16Dst, (uint8_t)u32Opcode, 0,
                           pstParam->model_operation.ctx->recv_ttl,
                           pstParam->model_operation.ctx->recv_rssi);

        /* Cập nhật đếm gói tin */
        APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
        if (stGuard.bLocked) {
            g_stInfo.u16LastSrc = u16Src;
            g_stInfo.u16LastDst = u16Dst;
            g_stInfo.u32RxCount++;
        }
        break;
    }
    default:
        break;
    }
}

/* ================== TASK Xá»¬ LÃ Sá»° KIá»†N MESH ================== */
static bool app_mesh_ModelHasAppKey(const esp_ble_mesh_model_t *pstModel, uint16_t u16AppIdx)
{
    for (size_t i = 0; i < ARRAY_SIZE(pstModel->keys); i++) {
        if (pstModel->keys[i] == u16AppIdx) {
            return true;
        }
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
    if (g_bGatewayConfigStarted &&
        (g_u8BindFailedCount != 0 || g_bGroupConfigFailed)) {
        g_bGatewayConfigStarted = false;
        g_bGatewayJoined = false;
    }
    if (g_bGatewayJoined) {
        LREP(TAG, "[GW] 3 AppKey da bind; cac model da subscribe Group 0x%04X.",
             g_u16ConfiguredGroup);
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
        { .pstModel = &g_astRootModels[2],      .u16CompanyId = ESP_BLE_MESH_CID_NVAL,
        .u16ModelId = ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV },
        { .pstModel = &g_astRootModels[3],      .u16CompanyId = ESP_BLE_MESH_CID_NVAL,
        .u16ModelId = ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI },
        { .pstModel = &g_astRootModels[4],      .u16CompanyId = ESP_BLE_MESH_CID_NVAL,
        .u16ModelId = ESP_BLE_MESH_MODEL_ID_HEALTH_CLI },  
        { .pstModel = &g_astVndModelsServer[0], .u16CompanyId = DF_MESH_VND_COMPANY_ID,
        .u16ModelId = DF_MESH_VND_MODEL_ID_SERVER },
    };

    g_u8BindPendingCount = 0;
    g_u8BindFailedCount = 0;

    for (size_t key = 0; key < ARRAY_SIZE(u16AppIdx); key++) {
        for (size_t model = 0; model < ARRAY_SIZE(astTargets); model++) {
            esp_ble_mesh_model_t *pstModel = astTargets[model].pstModel;
            uint16_t u16Cid = astTargets[model].u16CompanyId;
            uint16_t u16Mid = astTargets[model].u16ModelId;

            if (app_mesh_ModelHasAppKey(pstModel, u16AppIdx[key])) {
                continue;
            }

            esp_err_t err = esp_ble_mesh_node_bind_app_key_to_local_model(
                esp_ble_mesh_get_primary_element_address(),
                u16Cid,
                u16Mid,
                u16AppIdx[key]);

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
        if (stGuard.bLocked) {
            g_stInfo.bAppBound = true;
        }
    }
}

static void app_mesh_StartGatewayModelConfig(void)
{
    if (!g_bGatewayConfigStarted || g_u8AppKeyReadyMask != 0x07) {
        return;
    }
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
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return;
    }
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "Khong doc duoc cau hinh Gateway trong NVS: %s", esp_err_to_name(err));
        return;
    }

    uint16_t u16Value = 0;
    err = nvs_get_u16(hNvs, "group", &u16Value);
    if (err == ESP_OK && u16Value >= 0xC000 && u16Value <= 0xFEFF) {
        g_u16ConfiguredGroup = u16Value;
        g_stInfo.u16Group = u16Value;
    } else if (err != ESP_ERR_NVS_NOT_FOUND && err != ESP_OK) {
        LREP_ERROR(TAG, "Khong doc duoc Group Address: %s", esp_err_to_name(err));
    }

    err = nvs_get_u16(hNvs, "netidx", &u16Value);
    if (err == ESP_OK && u16Value <= 0x0FFF) {
        g_u16ProvisionedNetIdx = u16Value;
    } else if (err != ESP_ERR_NVS_NOT_FOUND && err != ESP_OK) {
        LREP_ERROR(TAG, "Khong doc duoc NetKey Index: %s", esp_err_to_name(err));
    }
    nvs_close(hNvs);
}

static int32_t app_mesh_GatewayJoinNetwork(uint16_t u16NetIdx)
{
    if (!g_bMeshInitialized || !esp_ble_mesh_node_is_provisioned()) {
        LREP_ERROR(TAG, "Node chua provisioned; lenh join khong the thay the Provisioner.");
        return ERROR_CODE_FAIL;
    }
    if (u16NetIdx > 0x0FFF || esp_ble_mesh_node_get_local_net_key(u16NetIdx) == NULL) {
        LREP_ERROR(TAG, "NetKey Index 0x%03X khong ton tai tren node.", u16NetIdx);
        return ERROR_CODE_NOT_FOUND;
    }
    if (g_bGatewayConfigStarted) {
        return g_bGatewayJoined ? ERROR_CODE_OK : ERROR_CODE_BUSY;
    }

    const uint8_t *pKeys[] = { HOMI_APP_KEY_0, HOMI_APP_KEY_1, HOMI_APP_KEY_2 };
    const uint16_t u16AppIdx[] = {
        DF_MESH_HOMI_APP_KEY_IDX_0,
        DF_MESH_HOMI_APP_KEY_IDX_1,
        DF_MESH_HOMI_APP_KEY_IDX_2,
    };
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
                LREP_ERROR(TAG, "AppKey Index 0x%03X da ton tai nhung key khong khop.", u16AppIdx[i]);
                g_bGatewayConfigStarted = false;
                return ERROR_CODE_INVALID_PARAM;
            }
            g_u8AppKeyReadyMask |= (uint8_t)(1U << i);
            continue;
        }

        esp_err_t err = esp_ble_mesh_node_add_local_app_key(pKeys[i], u16NetIdx, u16AppIdx[i]);
        if (err != ESP_OK) {
            LREP_ERROR(TAG, "Khong the them AppKey 0x%03X: %s", u16AppIdx[i], esp_err_to_name(err));
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

    if (g_u8AppKeyReadyMask == 0x07) {
        app_mesh_StartGatewayModelConfig();
    }
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
                    LREP(TAG, "===> Node DA duoc cap mang (NVS), Unicast=0x%04X <===", u16Addr);
                    g_stInfo.u16NetIdx = g_u16ProvisionedNetIdx;
                    app_mesh_GatewayJoinNetwork(g_u16ProvisionedNetIdx);
                } else {
                    LREP(TAG, "===> Node CHUA provisioned. Hay dung nRF Mesh de provision node '%s...'.", DF_MESH_DEVICE_NAME_PREFIX);
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
                LREP(TAG, "===> CAP MANG THANH CONG: NetIdx=0x%04X, Unicast=0x%04X <===", stEvt.u16Src, stEvt.u16Param);
                g_u16ProvisionedNetIdx = stEvt.u16Src;
                esp_err_t nvsErr;
                nvs_handle_t hNvs;
                nvsErr = nvs_open("meshgw", NVS_READWRITE, &hNvs);
                if (nvsErr == ESP_OK) {
                    nvsErr = nvs_set_u16(hNvs, "netidx", stEvt.u16Src);
                    if (nvsErr == ESP_OK) {
                        nvsErr = nvs_commit(hNvs);
                    }
                    nvs_close(hNvs);
                }
                if (nvsErr != ESP_OK) {
                    LREP_ERROR(TAG, "Khong luu duoc NetKey Index vao NVS: %s", esp_err_to_name(nvsErr));
                }
                app_mesh_GatewayJoinNetwork(stEvt.u16Src);
                break;
            }

            case MESH_EVT_PROV_RESET: {
                LREP_WARNING(TAG, "Node nhan lenh Reset -> Xoa mang va khoi dong lai!");
                app_common_DelayMs(300);
                esp_restart();
                break;
            }

            case MESH_EVT_APPKEY_ADD: {
                {
                    APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
                    if (stGuard.bLocked) {
                        g_stInfo.u16AppIdx = stEvt.u16Param;
                    }
                }
                LREP(TAG, "Da them AppKey 0x%04X", stEvt.u16Param);
                break;
            }

            case MESH_EVT_APP_BIND: {
                {
                    APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
                    if (stGuard.bLocked) {
                        g_stInfo.bAppBound = true;
                    }
                }
                LREP(TAG, "Da bind AppKey vao OnOff Server / Vendor Model!");
                break;
            }

            case MESH_EVT_SUB_ADD: {
                {
                    APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
                    if (stGuard.bLocked) {
                        g_stInfo.u16Group = stEvt.u16Param;
                    }
                }
                LREP(TAG, "Da subscribe vao Group 0x%04X", stEvt.u16Param);
                break;
            }

            case MESH_EVT_SUB_DEL: {
                {
                    APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
                    if (stGuard.bLocked) {
                        g_stInfo.u16Group = 0;
                    }
                }
                LREP(TAG, "Da huy subscribe Group 0x%04X", stEvt.u16Param);
                break;
            }

            case MESH_EVT_LOCAL_APPKEY_ADD: {
                int iKeyPos = -1;
                if (stEvt.u16Param == DF_MESH_HOMI_APP_KEY_IDX_0) iKeyPos = 0;
                if (stEvt.u16Param == DF_MESH_HOMI_APP_KEY_IDX_1) iKeyPos = 1;
                if (stEvt.u16Param == DF_MESH_HOMI_APP_KEY_IDX_2) iKeyPos = 2;
                if (stEvt.iErrCode != ESP_OK || iKeyPos < 0) {
                    LREP_ERROR(TAG, "Them AppKey local 0x%03X that bai (err=%d).",
                               stEvt.u16Param, stEvt.iErrCode);
                    g_bGatewayConfigStarted = false;
                    g_bGatewayJoined = false;
                    break;
                }
                g_u8AppKeyReadyMask |= (uint8_t)(1U << iKeyPos);
                LREP(TAG, "[GW] AppKey local 0x%03X san sang (mask=0x%02X).",
                     stEvt.u16Param, g_u8AppKeyReadyMask);
                app_mesh_StartGatewayModelConfig();
                break;
            }

            case MESH_EVT_LOCAL_APP_BIND: {
                /* Bỏ điều kiện lọc u16Dst == ESP_BLE_MESH_CID_NVAL để chấp nhận cả Vendor Model (CID 0x8888) */
                if (stEvt.u16Src != esp_ble_mesh_get_primary_element_address()) {
                    break;
                }
                if (g_u8BindPendingCount > 0) {
                    g_u8BindPendingCount--;
                }
                if (stEvt.iErrCode != ESP_OK) {
                    g_u8BindFailedCount++;
                    LREP_ERROR(TAG, "[GW] Bind AppKey 0x%03X vao model 0x%04X (CID 0x%04X) THAT BAI (err=%d)",
                               stEvt.u16Param, stEvt.u16ModelId, stEvt.u16Dst, stEvt.iErrCode);
                } else {
                    LREP(TAG, "[GW] Bound AppKey 0x%03X -> model 0x%04X (CID 0x%04X) THANH CONG.",
                         stEvt.u16Param, stEvt.u16ModelId, stEvt.u16Dst);
                }
                if (g_u8BindPendingCount == 0) {
                    APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
                    if (stGuard.bLocked) {
                        g_stInfo.bAppBound = (g_u8BindFailedCount == 0);
                    }
                }
                app_mesh_UpdateGatewayReady();
                break;
            }

            case MESH_EVT_LOCAL_SUB_ADD: {
                if (stEvt.u16Src != esp_ble_mesh_get_primary_element_address()) {
                    break;
                }
                if (g_u8GroupPendingCount > 0) {
                    g_u8GroupPendingCount--;
                }
                if (stEvt.iErrCode != ESP_OK) {
                    g_bGroupConfigFailed = true;
                    LREP_ERROR(TAG, "Subscribe model 0x%04X vao Group 0x%04X that bai (err=%d).",
                               stEvt.u16ModelId, stEvt.u16Param, stEvt.iErrCode);
                } else {
                    LREP(TAG, "[GW] Model 0x%04X subscribed Group 0x%04X.",
                         stEvt.u16ModelId, stEvt.u16Param);
                }
                if (g_u8GroupPendingCount == 0 && !g_bGroupConfigFailed) {
                    APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
                    if (stGuard.bLocked) {
                        g_stInfo.u16Group = g_u16ConfiguredGroup;
                    }
                }
                app_mesh_UpdateGatewayReady();
                break;
            }

            case MESH_EVT_LOCAL_SUB_DEL:
                if (stEvt.iErrCode != ESP_OK) {
                    LREP_WARNING(TAG, "Unsubscribe model 0x%04X Group 0x%04X loi (err=%d).",
                                 stEvt.u16ModelId, stEvt.u16Param, stEvt.iErrCode);
                }
                break;

            case MESH_EVT_ONOFF_SET: {
                LREP(TAG, " [RX ONOFF] Lenh OnOff=%u tu 0x%04X den 0x%04X (TTL=%u, RSSI=%d)",
                     stEvt.u8OnOff, stEvt.u16Src, stEvt.u16Dst, stEvt.u8Ttl, stEvt.s8Rssi);
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

                if (stEvt.u8OnOff) {
                    LREP_WARNING(TAG, ">>> CANH BAO: COI BAO CHAY DANG BAT ON! <<<");
                } else {
                    LREP(TAG, ">>> Coi bao chay da TAT OFF <<<");
                }
                break;
            }

            default:
                break;
            }
        }
    }
}

/* ================== KHá»žI Táº O BLUETOOTH ================== */
static esp_err_t app_mesh_BluetoothInit(void)
{
#if !CONFIG_IDF_TARGET_ESP32
    /* ESP32-S3 / C3 khong co Classic BT, giai phong bo nho */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
#endif

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        LREP_ERROR(TAG, "esp_bt_controller_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        LREP_ERROR(TAG, "esp_bt_controller_enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        LREP_ERROR(TAG, "esp_bluedroid_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        LREP_ERROR(TAG, "esp_bluedroid_enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/* ================== PUBLIC APIS ================== */

int32_t app_mesh_Init(void)
{
    if (g_bMeshInitialized) return ERROR_CODE_OK;

    LREP(TAG, "Khoi dong BLE Mesh Node...");

    g_hMeshMutex = xSemaphoreCreateMutex();
    if (g_hMeshMutex == NULL) {
        LREP_ERROR(TAG, "Khong tao duoc mutex!");
        return ERROR_CODE_NO_MEMORY;
    }

    g_hMeshEvtQ = xQueueCreate(DF_MESH_EVT_QUEUE_LEN, sizeof(st_mesh_evt_t));
    if (g_hMeshEvtQ == NULL) {
        LREP_ERROR(TAG, "Khong tao duoc queue!");
        return ERROR_CODE_NO_MEMORY;
    }

    memset(&g_stInfo, 0, sizeof(g_stInfo));
    g_stInfo.u32BootCount = 1;
    g_stInfo.u16NetIdx = g_u16ProvisionedNetIdx;
    g_stInfo.u16AppIdx = DF_MESH_HOMI_APP_KEY_IDX_0;
    app_mesh_LoadGatewayConfig();

    /* Tao task xu ly su kien truoc */
    if (xTaskCreate(app_mesh_Task, "fire_mesh_task", DF_MESH_TASK_STACK, NULL, DF_MESH_TASK_PRIO, &g_hMeshTask) != pdPASS) {
        LREP_ERROR(TAG, "Khong tao duoc task mesh!");
        return ERROR_CODE_FAIL;
    }

    /* Khoi tao Bluetooth Stack */
    if (app_mesh_BluetoothInit() != ESP_OK) return ERROR_CODE_FAIL;

    /* Lay dia chi MAC de dat ten node va UUID */
    uint8_t au8Mac[6] = {0};
    esp_read_mac(au8Mac, ESP_MAC_BT);
    memcpy(&g_au8DevUuid[2], au8Mac, sizeof(au8Mac));

    char acName[32];
    snprintf(acName, sizeof(acName), "%s%02X%02X", DF_MESH_DEVICE_NAME_PREFIX, au8Mac[4], au8Mac[5]);

    /* Tao Mutex quan ly danh sach thiet bi quet duoc */
    g_hScanMutex = xSemaphoreCreateMutex();
    if (g_hScanMutex == NULL) {
        LREP_WARNING(TAG, "Khong tao duoc scan mutex!");
    }

    /* Dang ky callbacks voi ESP BLE Mesh */
    esp_ble_mesh_register_prov_callback(app_mesh_ProvCb);
    esp_ble_mesh_register_config_client_callback(app_mesh_CfgCliCb);
    esp_ble_mesh_register_config_server_callback(app_mesh_CfgSrvCb);
    esp_ble_mesh_register_generic_server_callback(app_mesh_GenericSrvCb);
    esp_ble_mesh_register_generic_client_callback(app_mesh_GenericCliCb);
    esp_ble_mesh_register_custom_model_callback(app_mesh_VendorServerCb);
    esp_ble_mesh_register_health_client_callback(app_mesh_HealthCliCb);

    /* Khoi tao Stack Mesh */
    esp_err_t err = esp_ble_mesh_init(&g_stProv, &g_stComp);
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "esp_ble_mesh_init that bai: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    esp_ble_mesh_set_unprovisioned_device_name(acName);

    g_bMeshInitialized = true;

    /* Enable PB-ADV/PB-GATT so a Provisioner can provision this node. */
    err = esp_ble_mesh_node_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) {
        g_bMeshInitialized = false;
        LREP_ERROR(TAG, "esp_ble_mesh_node_prov_enable that bai: %s (err=%d)", esp_err_to_name(err), err);
        return ERROR_CODE_FAIL;
    }

    /* Kiem tra neu da duoc cap mang tu NVS (boot truoc do da join) */
    bool bAlreadyProvisioned = esp_ble_mesh_node_is_provisioned();
    if (bAlreadyProvisioned) {
        uint16_t u16Addr = esp_ble_mesh_get_primary_element_address();
        LREP(TAG, "[GW] Gateway DA CO thong tin mang tu NVS! Unicast=0x%04X", u16Addr);
        LREP(TAG, "[GW] Dang tu dong cau hinh AppKeys, model bindings va group subscription.");
        g_stInfo.u16NetIdx = g_u16ProvisionedNetIdx;
    } else {
        LREP_WARNING(TAG, "[GW] Node CHUA provisioned. Dung nRF Mesh de provision vao mang HOMI.");
    }

    /* Tao Task in thong tin dinh ky moi 3 giay */
    if (xTaskCreate(app_mesh_ScannerTask, "mesh_gw_task", 4096, NULL, 3, &g_hScanPrintTask) != pdPASS) {
        LREP_WARNING(TAG, "Khong tao duoc task gateway!");
    }

    LREP(TAG, "==========================================================");
    LREP(TAG, "  HOMI GATEWAY NODE DA KHOI DONG!");
    LREP(TAG, "  Che do   : BLE Mesh Node (KHONG PHAI Provisioner)");
    LREP(TAG, "  Vendor   : Model 0x%04X%04X", DF_MESH_VND_COMPANY_ID, DF_MESH_VND_MODEL_ID_SERVER);
    LREP(TAG, "  Group    : 0x%04X (co the doi bang meshgw sub <group>)", g_u16ConfiguredGroup);
    LREP(TAG, "  Lenh xem : 'mesh'  |  'meshgw status'");
    LREP(TAG, "==========================================================");
    return ERROR_CODE_OK;
}

void app_mesh_PrintStatus(void)
{
    if (!g_bMeshInitialized) {
        printf("BLE Mesh chua duoc khoi tao.\n");
        return;
    }

    bool bProv = esp_ble_mesh_node_is_provisioned();
    uint16_t u16Addr = bProv ? esp_ble_mesh_get_primary_element_address() : 0;

    st_mesh_info_t stSnap;
    {
        APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
        if (!stGuard.bLocked) {
            printf("Mesh dang ban!\n");
            return;
        }
        stSnap = g_stInfo;
    }

    printf("\n========================================\n");
    printf("       THONG TIN BLE MESH FIRE NODE     \n");
    printf("========================================\n");
    printf("Trang thai cap mang : %s\n", bProv ? "DA CAP MANG (PROVISIONED)" : "CHUA CAP MANG (UNPROVISIONED)");
    printf("Unicast Address     : 0x%04X\n", u16Addr);
    printf("NetIdx / AppIdx     : 0x%04X / 0x%04X\n", stSnap.u16NetIdx, stSnap.u16AppIdx);
    printf("AppKey bound        : %s\n", stSnap.bAppBound ? "CO" : "CHUA");
    if (stSnap.u16Group != 0) {
        printf("Group Subscribed    : 0x%04X\n", stSnap.u16Group);
    } else {
        printf("Group Subscribed    : (Chua subscribe)\n");
    }
    printf("Trang thai Bao chay : %s (%u)\n", stSnap.u8AlarmState ? "DANG BAO DONG" : "BINH THUONG", stSnap.u8AlarmState);
    printf("Goi tin RX / TX     : %u / %u (Drop: %u)\n", (unsigned)stSnap.u32RxCount, (unsigned)stSnap.u32TxCount, (unsigned)g_u32DropEvt);
    printf("Nguon/Dich gan nhat : 0x%04X -> 0x%04X\n", stSnap.u16LastSrc, stSnap.u16LastDst);
    printf("Relay feature       : %s\n", DF_MESH_RELAY ? "BAT (ENABLED)" : "TAT (DISABLED)");
    printf("========================================\n\n");
}

void app_mesh_PrintLog(void)
{
    if (!g_bMeshInitialized) {
        printf("BLE Mesh chua duoc khoi tao.\n");
        return;
    }

    st_mesh_log_t astSnap[DF_MESH_LOG_DEPTH];
    uint32_t u32Count = 0;
    uint32_t u32Start = 0;
    {
        APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
        if (!stGuard.bLocked) return;
        memcpy(astSnap, g_astLog, sizeof(astSnap));
        u32Count = g_u32LogCount;
        u32Start = (g_u32LogCount < DF_MESH_LOG_DEPTH) ? 0 : g_u32LogHead;
    }

    printf("\n--- LICH SU %u BAN TIN GAN NHAT ---\n", (unsigned)u32Count);
    printf("%-3s | %-8s | %-8s | %-5s | %-6s | %-4s | %-5s\n", "#", "Nguon", "Dich", "Cmd", "State", "TTL", "RSSI");
    for (uint32_t i = 0; i < u32Count; i++) {
        const st_mesh_log_t *pst = &astSnap[(u32Start + i) % DF_MESH_LOG_DEPTH];
        printf("%-3u | 0x%04X   | 0x%04X   | %-5u | %-6u | %-4u | %-5d\n",
               (unsigned)i, pst->u16Src, pst->u16Dst, pst->u8Cmd, pst->u8State, pst->u8Ttl, pst->s8Rssi);
    }
    printf("------------------------------------\n\n");
}

int32_t app_mesh_Reset(void)
{
    if (!g_bMeshInitialized) return ERROR_CODE_FAIL;

    LREP_WARNING(TAG, "Tien hanh Reset BLE Mesh (xoa du lieu cap mang)...");
    esp_err_t err = esp_ble_mesh_node_local_reset();
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "node_local_reset that bai: %s", esp_err_to_name(err));
    }
    app_common_DelayMs(500);
    esp_restart();
    return ERROR_CODE_OK;
}

int32_t app_mesh_SendOnOff(uint16_t u16DstAddr, uint8_t u8OnOff)
{
    if (!g_bMeshInitialized) {
        printf("BLE Mesh chua khoi tao!\n");
        return ERROR_CODE_FAIL;
    }

    uint16_t u16NetIdx = 0;
    uint16_t u16AppIdx = 0;
    {
        APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
        if (!stGuard.bLocked) return ERROR_CODE_TIMEOUT;
        u16NetIdx = g_stInfo.u16NetIdx;
        u16AppIdx = g_stInfo.u16AppIdx;
    }

    esp_ble_mesh_generic_client_set_state_t stSetParam = {
        .onoff_set.onoff = (u8OnOff ? 1 : 0),
        .onoff_set.op_en = false,
    };

    esp_ble_mesh_client_common_param_t stCommonParam = {
        .opcode      = ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK,
        .model       = &g_astRootModels[3], /* OnOff Client */
        .msg_timeout = 0,
        .ctx.net_idx   = u16NetIdx,
        .ctx.app_idx   = u16AppIdx,
        .ctx.addr      = u16DstAddr,
        .ctx.send_ttl  = 7,
    };

    esp_err_t err = esp_ble_mesh_generic_client_set_state(&stCommonParam, &stSetParam);
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "Gui Generic OnOff that bai: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    {
        APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
        if (stGuard.bLocked) g_stInfo.u32TxCount++;
    }

    LREP(TAG, "[TX ONOFF] Gui OnOff=%u toi 0x%04X", u8OnOff, u16DstAddr);
    return ERROR_CODE_OK;
}

int32_t app_mesh_SendFireAlarm(uint16_t u16DstAddr, uint8_t u8AlarmState, int16_t s16Temp, uint16_t u16Smoke)
{
    if (!g_bMeshInitialized) {
        printf("BLE Mesh chua khoi tao!\n");
        return ERROR_CODE_FAIL;
    }

    static uint16_t s_u16PktCounter = 0;
    s_u16PktCounter++;

    st_vendor_custom_data_t stMsg = {
        .u8Cmd          = u8AlarmState ? DF_CMD_FIRE_ALARM_TRIGGER : DF_CMD_FIRE_ALARM_NORMAL,
        .u16PacketId    = s_u16PktCounter,
        .s16Temperature = s16Temp,
        .u16SmokeVal    = u16Smoke,
        .u8AlarmState   = u8AlarmState ? 1 : 0,
    };
    snprintf(stMsg.acDeviceTag, sizeof(stMsg.acDeviceTag), "FIRE_%04X", esp_ble_mesh_get_primary_element_address());

    uint16_t u16NetIdx = 0;
    uint16_t u16AppIdx = 0;
    {
        APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
        if (!stGuard.bLocked) return ERROR_CODE_TIMEOUT;
        u16NetIdx = g_stInfo.u16NetIdx;
        u16AppIdx = g_stInfo.u16AppIdx;
    }

    esp_ble_mesh_msg_ctx_t ctx = {
        .net_idx  = u16NetIdx,
        .app_idx  = u16AppIdx,
        .addr     = u16DstAddr,
        .send_ttl = 7,
    };

    /* Gui qua Vendor Model Server */
    esp_err_t err = esp_ble_mesh_server_model_send_msg(
        &g_astVndModelsServer[0],
        &ctx,
        DF_MESH_VND_OP_SET,
        sizeof(stMsg),
        (uint8_t *)&stMsg
    );

    if (err != ESP_OK) {
        LREP_ERROR(TAG, "Gui ban tin Fire Alarm that bai: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    {
        APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
        if (stGuard.bLocked) {
            g_stInfo.u8AlarmState = u8AlarmState;
            g_stInfo.u32TxCount++;
        }
    }

    LREP(TAG, "[TX FIRE] Da phat canh bao toi 0x%04X: State=%u, Temp=%.1foC, Smoke=%u PPM",
         u16DstAddr, u8AlarmState, (float)s16Temp / 10.0f, u16Smoke);

    /* Dong thoi phat lenh Generic OnOff Set toi dia chi do de dong bo chuong */
    app_mesh_SendOnOff(u16DstAddr, u8AlarmState);

    return ERROR_CODE_OK;
}

int32_t app_mesh_SendVendorRaw(uint16_t u16Dst, uint8_t u8OpIdx, const uint8_t *pu8Data, uint16_t u16Len)
{
    if (!g_bMeshInitialized) {
        printf("BLE Mesh chua khoi tao!\n");
        return ERROR_CODE_FAIL;
    }
    /* 8 byte tham so + 3 byte opcode = 11 byte, vua mot goi khong phan manh */
    if (u8OpIdx > 0x3F || u16Len > 8 || (u16Len > 0 && pu8Data == NULL)) {
        return ERROR_CODE_INVALID_PARAM;
    }

    uint16_t u16NetIdx = 0;
    uint16_t u16AppIdx = 0;
    {
        APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
        if (!stGuard.bLocked) return ERROR_CODE_TIMEOUT;
        u16NetIdx = g_stInfo.u16NetIdx;
        u16AppIdx = g_stInfo.u16AppIdx;
    }

    esp_ble_mesh_msg_ctx_t ctx = {
        .net_idx  = u16NetIdx,
        .app_idx  = u16AppIdx,
        .addr     = u16Dst,
        .send_ttl = 7,
    };
    uint32_t u32Op = ESP_BLE_MESH_MODEL_OP_3(u8OpIdx, DF_MESH_VND_COMPANY_ID);

    esp_err_t err = esp_ble_mesh_server_model_send_msg(&g_astVndModelsServer[0], &ctx,
                                                       u32Op, u16Len, (uint8_t *)pu8Data);
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "[TX VND] that bai: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }

    {
        APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
        if (stGuard.bLocked) g_stInfo.u32TxCount++;
    }

    LREP(TAG, "[TX VND] -> 0x%04X op=0x%06lX len=%u", u16Dst, (unsigned long)u32Op, (unsigned)u16Len);
    return ERROR_CODE_OK;
}

bool app_mesh_IsProvisioned(void)
{
    return g_bMeshInitialized && esp_ble_mesh_node_is_provisioned();
}

uint8_t app_mesh_GetAlarmState(void)
{
    return g_stInfo.u8AlarmState;
}

void app_mesh_SetAlarmState(uint8_t u8AlarmState)
{
    APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
    if (stGuard.bLocked) {
        g_stInfo.u8AlarmState = u8AlarmState;
    }
}

/* ================== NHIá»†M Vá»¤ VÃ€ HÃ€M IN BÃO CÃO QUÃ‰T 3 GIÃ‚Y ================== */

void app_mesh_PrintScannedDevices(void)
{
    if (g_hScanMutex == NULL) return;
    if (xSemaphoreTake(g_hScanMutex, pdMS_TO_TICKS(500)) != pdTRUE) return;

    int64_t s64NowUs = esp_timer_get_time();
    printf("\n========================================================================================================================\n");
    printf(" [BLE MESH SCANNER] (Chu ky 3s) | Phat hien: %u thiet bi | Che do: Chi quet (Chua ket noi)\n", (unsigned)g_u32ScannedCount);
    printf("========================================================================================================================\n");

    if (g_u32ScannedCount == 0) {
        printf("  >> Dang lien tuc quet cac goi tin BLE Mesh xung quanh... (Chua tim thay thiet bi nao)\n");
    } else {
        printf("  %-3s | %-17s | %-6s | %-8s | %-26s | %-5s | %-9s | %s\n",
               "#", "MAC Address", "Type", "RSSI", "Loai thiet bi / PDU", "Pkts", "Last Seen", "Device UUID");
        printf("  ----+-------------------+--------+----------+----------------------------+-------+-----------+---------------------------------\n");
        for (uint32_t i = 0; i < g_u32ScannedCount; i++) {
            st_mesh_scanned_dev_t *pDev = &g_astScannedDevs[i];
            float fDeltaSec = (float)(s64NowUs - pDev->s64LastSeenUs) / 1000000.0f;

            char acUuidStr[36] = "N/A";
            if (pDev->bHasUuid) {
                snprintf(acUuidStr, sizeof(acUuidStr),
                         "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
                         pDev->au8Uuid[0], pDev->au8Uuid[1], pDev->au8Uuid[2], pDev->au8Uuid[3],
                         pDev->au8Uuid[4], pDev->au8Uuid[5], pDev->au8Uuid[6], pDev->au8Uuid[7],
                         pDev->au8Uuid[8], pDev->au8Uuid[9], pDev->au8Uuid[10], pDev->au8Uuid[11],
                         pDev->au8Uuid[12], pDev->au8Uuid[13], pDev->au8Uuid[14], pDev->au8Uuid[15]);
            }

            printf("  %-3u | %02X:%02X:%02X:%02X:%02X:%02X | %-6s | %-4d dBm | %-26s | %-5u | %4.1fs ago | %s\n",
                   (unsigned)(i + 1),
                   pDev->au8Mac[0], pDev->au8Mac[1], pDev->au8Mac[2],
                   pDev->au8Mac[3], pDev->au8Mac[4], pDev->au8Mac[5],
                   pDev->u8AddrType ? "RANDOM" : "PUBLIC",
                   pDev->s8Rssi,
                   pDev->acDeviceType,
                   (unsigned)pDev->u32PktCount,
                   fDeltaSec,
                   acUuidStr);
        }
    }
    printf("========================================================================================================================\n\n");

    xSemaphoreGive(g_hScanMutex);
}

void app_mesh_ClearScannedDevices(void)
{
    if (g_hScanMutex == NULL) return;
    if (xSemaphoreTake(g_hScanMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        memset(g_astScannedDevs, 0, sizeof(g_astScannedDevs));
        g_u32ScannedCount = 0;
        xSemaphoreGive(g_hScanMutex);
    }
}

uint32_t app_mesh_GetScannedDeviceCount(void)
{
    return g_u32ScannedCount;
}

bool app_mesh_GetGatewayJoined(void)
{
    return g_bGatewayJoined;
}

#if 0
/*
 * app_mesh_GatewayJoinNetwork()
 *
 * Má»¤C ÄÃCH: Join máº¡ng HOMI Ä‘Ã£ provision sáºµn.
 *
 * LÆ¯U Ã QUAN TRá»ŒNG (ESP-IDF BLE Mesh API):
 *   - esp_ble_mesh_node_add_local_net_key() CHá»ˆ hoáº¡t Ä‘á»™ng SAU KHI Ä‘Ã£ provisioned.
 *   - esp_ble_mesh_node_bind_app_key_to_local_model() dÃ¹ng element_addr (Unicast).
 *
 * LUá»’NG Sá»¬ Dá»¤NG:
 *   CÃ¡ch 1 (KhuyÃªn dÃ¹ng - Chuáº©n BLE Mesh):
 *     1. DÃ¹ng nRF Mesh App / S3 Provisioner vá»›i NetKey HOMI Ä‘á»ƒ provision gateway nÃ y.
 *     2. Sau khi cÃ³ Unicast, gá»i 'meshgw join' Ä‘á»ƒ inject thÃªm AppKey 1, 2.
 *     3. Gateway láº¯ng nghe má»i PDU trong máº¡ng HOMI.
 *
 *   CÃ¡ch 2 (Nhanh - náº¿u Ä‘Ã£ cÃ³ Unicast tá»« trÆ°á»›c):
 *     Restart -> 'meshgw join' ngay sau khi boot.
 */
static int32_t app_mesh_GatewayJoinNetwork(const uint8_t *pu8NetKey, uint16_t u16NetIdx,
                                           const uint8_t *pu8AppKey0, uint16_t u16AppIdx0,
                                           const uint8_t *pu8AppKey1, uint16_t u16AppIdx1,
                                           const uint8_t *pu8AppKey2, uint16_t u16AppIdx2)
{
    if (!g_bMeshInitialized) {
        printf("[GW] BLE Mesh chua khoi tao!\n");
        return ERROR_CODE_FAIL;
    }

    bool bProvisioned = esp_ble_mesh_node_is_provisioned();

    if (!bProvisioned) {
        /* Chua provisioned - chi co the inject key sau khi da provisioned */
        printf("\n[GW] ============================================================\n");
        printf("[GW] CHUA PROVISIONED - Chua the inject key!\n");
        printf("[GW] Vui long thuc hien theo thu tu sau:\n");
        printf("[GW]   1. Mo nRF Mesh App (Android/iOS)\n");
        printf("[GW]   2. Them mang HOMI voi NetKey:\n");
        printf("[GW]   3. Scan va Provision thiet bi nay\n");
        printf("[GW]   4. Sau khi provisioned, chay lai lenh: meshgw join\n");
        printf("[GW] ============================================================\n\n");
        return ERROR_CODE_FAIL;
    }

    /* Da provisioned - lay Unicast address de dung cho bind */
    uint16_t u16Unicast = esp_ble_mesh_get_primary_element_address();

    LREP(TAG, "[GW] ======================================");
    LREP(TAG, "[GW] Bat dau inject AppKey mang HOMI...");
    LREP(TAG, "[GW] Unicast = 0x%04X | NetKeyIdx = 0x%04X", u16Unicast, u16NetIdx);
    LREP(TAG, "[GW] AppKeyIdx = 0x%04X, 0x%04X, 0x%04X", u16AppIdx0, u16AppIdx1, u16AppIdx2);

    esp_err_t err;

    /* --- Them AppKey 1 va 2 (AppKey 0 thuong da duoc provisioner cap roi) --- */
    /* AppKey 0 - co the da co, thu them lai (bo qua neu trung) */
    err = esp_ble_mesh_node_add_local_app_key(pu8AppKey0, u16NetIdx, u16AppIdx0);
    if (err == ESP_OK) {
        LREP(TAG, "[GW] Them AppKey[0] (idx=0x%04X) thanh cong!", u16AppIdx0);
    } else {
        /* Co the da ton tai tu provisioner - khong phai loi nghiem trong */
        LREP_WARNING(TAG, "[GW] AppKey[0]: %s (co the da co san)", esp_err_to_name(err));
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    /* AppKey 1 */
    err = esp_ble_mesh_node_add_local_app_key(pu8AppKey1, u16NetIdx, u16AppIdx1);
    if (err == ESP_OK) {
        LREP(TAG, "[GW] Them AppKey[1] (idx=0x%04X) thanh cong!", u16AppIdx1);
    } else {
        LREP_WARNING(TAG, "[GW] AppKey[1]: %s", esp_err_to_name(err));
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    /* AppKey 2 */
    err = esp_ble_mesh_node_add_local_app_key(pu8AppKey2, u16NetIdx, u16AppIdx2);
    if (err == ESP_OK) {
        LREP(TAG, "[GW] Them AppKey[2] (idx=0x%04X) thanh cong!", u16AppIdx2);
    } else {
        LREP_WARNING(TAG, "[GW] AppKey[2]: %s", esp_err_to_name(err));
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    /* --- Bind AppKey[0] vao cac model (dung Unicast address) --- */
    /* Generic OnOff Server */
    err = esp_ble_mesh_node_bind_app_key_to_local_model(
        u16Unicast,              /* element_addr = Unicast cua primary element */
        ESP_BLE_MESH_CID_NVAL,  /* SIG model */
        ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV,
        u16AppIdx0
    );
    if (err == ESP_OK) {
        LREP(TAG, "[GW] Bind AppKey[0] -> OnOff Server OK");
    } else {
        LREP_WARNING(TAG, "[GW] Bind OnOff Srv: %s", esp_err_to_name(err));
    }

    /* Generic OnOff Client */
    err = esp_ble_mesh_node_bind_app_key_to_local_model(
        u16Unicast,
        ESP_BLE_MESH_CID_NVAL,
        ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI,
        u16AppIdx0
    );
    if (err == ESP_OK) {
        LREP(TAG, "[GW] Bind AppKey[0] -> OnOff Client OK");
    } else {
        LREP_WARNING(TAG, "[GW] Bind OnOff Cli: %s", esp_err_to_name(err));
    }

    /* Vendor Server Model */
    err = esp_ble_mesh_node_bind_app_key_to_local_model(
        u16Unicast,
        DF_MESH_VND_COMPANY_ID,
        DF_MESH_VND_MODEL_ID_SERVER,
        u16AppIdx0
    );
    if (err == ESP_OK) {
        LREP(TAG, "[GW] Bind AppKey[0] -> Vendor Server OK");
    } else {
        LREP_WARNING(TAG, "[GW] Bind Vendor Srv: %s", esp_err_to_name(err));
    }

    /* Cap nhat trang thai noi bo */
    {
        APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
        if (stGuard.bLocked) {
            g_stInfo.bProvisioned = true;
            g_stInfo.u16Unicast   = u16Unicast;
            g_stInfo.u16NetIdx    = u16NetIdx;
            g_stInfo.u16AppIdx    = u16AppIdx0;
            g_stInfo.bAppBound    = true;
        }
    }

    g_bGatewayJoined = true;

    LREP(TAG, "[GW] ======================================");
    LREP(TAG, "[GW] HOMI GATEWAY JOIN THANH CONG!");
    LREP(TAG, "[GW] Unicast    : 0x%04X", u16Unicast);
    LREP(TAG, "[GW] NetKey Idx : 0x%04X", u16NetIdx);
    LREP(TAG, "[GW] AppKey Idx : 0x%04X (+ 0x%04X, 0x%04X)", u16AppIdx0, u16AppIdx1, u16AppIdx2);
    LREP(TAG, "[GW] Gateway dang lang nghe PDU trong mang HOMI.");
    LREP(TAG, "[GW] ======================================");

    return ERROR_CODE_OK;
}

int32_t app_mesh_GatewayJoin(void)
{
    return app_mesh_GatewayJoinNetwork(
        HOMI_NET_KEY, HOMI_NET_KEY_IDX,
        HOMI_APP_KEY_0, HOMI_APP_KEY_IDX_0,
        HOMI_APP_KEY_1, HOMI_APP_KEY_IDX_1,
        HOMI_APP_KEY_2, HOMI_APP_KEY_IDX_2
    );
}
#endif

static esp_err_t app_mesh_SubscribeOneModel(esp_ble_mesh_model_t *pstModel, uint16_t u16Group)
{
    if (pstModel == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Đã subscribe rồi thì bỏ qua */
    if (esp_ble_mesh_is_model_subscribed_to_group(pstModel, u16Group) != NULL) {
        return ESP_OK;
    }

    uint16_t company_id;
    uint16_t model_id;

    if (pstModel == &g_astVndModelsServer[0]) {
        /* Vendor Model HOMI: 0x88880000 */
        company_id = DF_MESH_VND_COMPANY_ID;      // 0x8888
        model_id   = DF_MESH_VND_MODEL_ID_SERVER; // 0x0000
    } else {
        /* SIG Model (OnOff ...) */
        company_id = ESP_BLE_MESH_CID_NVAL;
        model_id   = pstModel->model_id;
    }

    return esp_ble_mesh_model_subscribe_group_addr(
        esp_ble_mesh_get_primary_element_address(),
        company_id,
        model_id,
        u16Group
    );
}

static int32_t app_mesh_SubscribeGroup(uint16_t u16GroupAddr, bool bPersist)
{
    if (!g_bMeshInitialized || !esp_ble_mesh_node_is_provisioned()) {
        LREP_ERROR(TAG, "Can provision node truoc khi subscribe Group.");
        return ERROR_CODE_FAIL;
    }
    if (u16GroupAddr < 0xC000 || u16GroupAddr > 0xFEFF) {
        LREP_ERROR(TAG, "Group Address 0x%04X khong hop le (phai trong 0xC000-0xFEFF).",
                   u16GroupAddr);
        return ERROR_CODE_INVALID_PARAM;
    }

    /* CHỈ SUBSCRIBE CHO SERVER MODELS - KHÔNG SUBSCRIBE CLIENT MODEL */
    esp_ble_mesh_model_t *pModels[] = {
        &g_astRootModels[2],      /* Generic OnOff Server */
        &g_astVndModelsServer[0], /* Vendor Model Server (0x8888) */
    };

    uint16_t u16OldGroup = g_u16ConfiguredGroup;
    g_u8GroupPendingCount = 0;
    g_bGroupConfigFailed = false;

    if (u16OldGroup != u16GroupAddr && u16OldGroup >= 0xC000 && u16OldGroup <= 0xFEFF) {
        for (size_t i = 0; i < ARRAY_SIZE(pModels); i++) {
            esp_ble_mesh_model_t *pstModel = pModels[i];
            if (esp_ble_mesh_is_model_subscribed_to_group(pstModel, u16OldGroup) != NULL) {
                esp_err_t err = esp_ble_mesh_model_unsubscribe_group_addr(
                    esp_ble_mesh_get_primary_element_address(),
                    pstModel == &g_astVndModelsServer[0] ? DF_MESH_VND_COMPANY_ID : ESP_BLE_MESH_CID_NVAL,
                    pstModel == &g_astVndModelsServer[0] ? DF_MESH_VND_MODEL_ID_SERVER : pstModel->model_id,
                    u16OldGroup);
                if (err != ESP_OK) {
                    LREP_ERROR(TAG, "Khong the queue unsubscribe Group 0x%04X: %s",
                               u16OldGroup, esp_err_to_name(err));
                    return ERROR_CODE_FAIL;
                }
            }
        }
    }

    for (size_t i = 0; i < ARRAY_SIZE(pModels); i++) {
        bool bAlreadySubscribed =
            esp_ble_mesh_is_model_subscribed_to_group(pModels[i], u16GroupAddr) != NULL;
        esp_err_t err = app_mesh_SubscribeOneModel(pModels[i], u16GroupAddr);
        if (err == ESP_OK) {
            if (!bAlreadySubscribed) {
                g_u8GroupPendingCount++;
            }
        } else {
            g_bGroupConfigFailed = true;
            LREP_ERROR(TAG, "Khong the queue subscribe model 0x%04X vao Group 0x%04X: %s",
                       pModels[i]->model_id, u16GroupAddr, esp_err_to_name(err));
            return ERROR_CODE_FAIL;
        }
    }

    if (bPersist) {
        nvs_handle_t hNvs;
        esp_err_t err = nvs_open("meshgw", NVS_READWRITE, &hNvs);
        if (err == ESP_OK) {
            err = nvs_set_u16(hNvs, "group", u16GroupAddr);
            if (err == ESP_OK) {
                err = nvs_commit(hNvs);
            }
            nvs_close(hNvs);
        }
        if (err != ESP_OK) {
            LREP_ERROR(TAG, "Khong luu duoc Group Address vao NVS: %s", esp_err_to_name(err));
            return ERROR_CODE_FAIL;
        }
    }

    g_u16ConfiguredGroup = u16GroupAddr;
    if (g_u8GroupPendingCount == 0) {
        APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
        if (stGuard.bLocked) {
            g_stInfo.u16Group = u16GroupAddr;
        }
    }
    app_mesh_UpdateGatewayReady();
    return ERROR_CODE_OK;
}

int32_t app_mesh_GatewayJoin(void)
{
    return app_mesh_GatewayJoinNetwork(g_u16ProvisionedNetIdx);
}

int32_t app_mesh_SetGroupAddress(uint16_t u16GroupAddr)
{
    return app_mesh_SubscribeGroup(u16GroupAddr, true);
}

uint16_t app_mesh_GetGroupAddress(void)
{
    return g_u16ConfiguredGroup;
}

static void app_mesh_ScannerTask(void *pvParameters)
{
    /* Dinh ky 3 giay in trang thai Gateway + Mesh */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000));

        bool bProv  = esp_ble_mesh_node_is_provisioned();
        bool bJoined = g_bGatewayJoined;
        uint16_t u16Addr = bProv ? esp_ble_mesh_get_primary_element_address() : 0;

        printf("\n--- [HOMI GW] Trang thai (%lld ms) ---\n",
               (long long)(esp_timer_get_time() / 1000));
        printf("  Gateway config : %s\n", bJoined ? "HOAN TAT" : "DANG CHO / CO LOI");
        printf("  Node prov      : %s\n", bProv ? "DA PROVISIONED" : "CHUA PROVISIONED");
        if (bProv) {
            printf("  Unicast addr   : 0x%04X\n", u16Addr);
        }

        uint32_t u32RxCount, u32TxCount;
        {
            APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
            if (stGuard.bLocked) {
                u32RxCount = g_stInfo.u32RxCount;
                u32TxCount = g_stInfo.u32TxCount;
            } else {
                u32RxCount = u32TxCount = 0;
            }
        }
        printf("  RX / TX count  : %u / %u (Drop: %u)\n",
               (unsigned)u32RxCount, (unsigned)u32TxCount, (unsigned)g_u32DropEvt);
        printf("---------------------------------------\n\n");
    }
}

int32_t app_mesh_HealthFaultGet(uint16_t u16Dst, uint16_t u16CompanyId)
{
    if (!g_bMeshInitialized || !esp_ble_mesh_node_is_provisioned()) {
        return ERROR_CODE_FAIL;
    }

    uint16_t u16NetIdx = 0, u16AppIdx = 0;
    {
        APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
        if (!stGuard.bLocked) return ERROR_CODE_TIMEOUT;
        u16NetIdx = g_stInfo.u16NetIdx;
        u16AppIdx = g_stInfo.u16AppIdx;
    }

    esp_ble_mesh_client_common_param_t common = {0};
    common.opcode       = ESP_BLE_MESH_MODEL_OP_HEALTH_FAULT_GET;
    common.model        = &g_astRootModels[4];
    common.ctx.net_idx  = u16NetIdx;
    common.ctx.app_idx  = u16AppIdx;
    common.ctx.addr     = u16Dst;
    common.ctx.send_ttl = 7;
    common.msg_timeout  = 4000;

    esp_ble_mesh_health_client_get_state_t get = {0};
    get.fault_get.company_id = u16CompanyId;

    esp_err_t err = esp_ble_mesh_health_client_get_state(&common, &get);
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "Health Fault Get that bai: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }
    LREP(TAG, "[HEALTH TX] Fault Get -> 0x%04X (CID=0x%04X)", u16Dst, u16CompanyId);
    return ERROR_CODE_OK;
}
int32_t app_mesh_HealthAttentionSet(uint16_t u16Dst, uint8_t u8Attention)
{
    if (!g_bMeshInitialized || !esp_ble_mesh_node_is_provisioned()) {
        return ERROR_CODE_FAIL;
    }

    uint16_t u16NetIdx = 0, u16AppIdx = 0;
    {
        APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
        if (!stGuard.bLocked) return ERROR_CODE_TIMEOUT;
        u16NetIdx = g_stInfo.u16NetIdx;
        u16AppIdx = g_stInfo.u16AppIdx;
    }

    esp_ble_mesh_client_common_param_t common = {0};
    common.opcode       = ESP_BLE_MESH_MODEL_OP_ATTENTION_SET_UNACK;  /* đúng tên IDF 5.x */
    common.model        = &g_astRootModels[4];
    common.ctx.net_idx  = u16NetIdx;
    common.ctx.app_idx  = u16AppIdx;
    common.ctx.addr     = u16Dst;
    common.ctx.send_ttl = 7;
    common.msg_timeout  = 4000;

    esp_ble_mesh_health_client_set_state_t set = {0};
    set.attention_set.attention = u8Attention;

    esp_err_t err = esp_ble_mesh_health_client_set_state(&common, &set);
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "Health Attention Set that bai: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }
    LREP(TAG, "[HEALTH TX] Attention Set=%u -> 0x%04X", u8Attention, u16Dst);
    return ERROR_CODE_OK;
}

int32_t app_mesh_HealthAttentionGet(uint16_t u16Dst)
{
    if (!g_bMeshInitialized || !esp_ble_mesh_node_is_provisioned()) {
        return ERROR_CODE_FAIL;
    }

    uint16_t u16NetIdx = 0, u16AppIdx = 0;
    {
        APP_LOCK_GUARD(stGuard, g_hMeshMutex, MESH_INFO_MUTEX_NAME, DF_LOCK_TIMEOUT_MS);
        if (!stGuard.bLocked) return ERROR_CODE_TIMEOUT;
        u16NetIdx = g_stInfo.u16NetIdx;
        u16AppIdx = g_stInfo.u16AppIdx;
    }

    esp_ble_mesh_client_common_param_t common = {0};
    common.opcode       = ESP_BLE_MESH_MODEL_OP_ATTENTION_GET;  /* đúng tên IDF 5.x */
    common.model        = &g_astRootModels[4];
    common.ctx.net_idx  = u16NetIdx;
    common.ctx.app_idx  = u16AppIdx;
    common.ctx.addr     = u16Dst;
    common.ctx.send_ttl = 7;
    common.msg_timeout  = 4000;

    esp_ble_mesh_health_client_get_state_t get = {0};

    esp_err_t err = esp_ble_mesh_health_client_get_state(&common, &get);
    if (err != ESP_OK) {
        LREP_ERROR(TAG, "Health Attention Get that bai: %s", esp_err_to_name(err));
        return ERROR_CODE_FAIL;
    }
    LREP(TAG, "[HEALTH TX] Attention Get -> 0x%04X", u16Dst);
    return ERROR_CODE_OK;
}