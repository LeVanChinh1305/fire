#pragma once

#include "app_common.h"
#include "app_mesh_vendor.h"

/**
 * @brief Khởi tạo HOMI Gateway Node (BLE Mesh Node + GATT Proxy):
 *        Bluetooth Controller, Bluedroid, ESP BLE Mesh Stack.
 *        Không còn là Provisioner, chỉ là Node thường trong mạng HOMI.
 */
int32_t app_mesh_Init(void);

/** @brief In trạng thái chi tiết của Mesh Node (Unicast, Subnet, AppKey, Group, OnOff) */
void app_mesh_PrintStatus(void);

/** @brief In lịch sử log các sự kiện / bản tin gần nhất */
void app_mesh_PrintLog(void);

/** @brief Xóa thông tin cấp mạng (Reset Node) về trạng thái unprovisioned */
int32_t app_mesh_Reset(void);

/**
 * @brief Gửi lệnh Generic OnOff tới Node hoặc Group
 * @param u16DstAddr Địa chỉ đích (ví dụ: 0x0002 hoặc 0xC000)
 * @param u8OnOff 0 (OFF) hoặc 1 (ON)
 */
int32_t app_mesh_SendOnOff(uint16_t u16DstAddr, uint8_t u8OnOff);

/**
 * @brief Gửi bản tin cảnh báo cháy (Vendor Model) tới Node hoặc Group
 * @param u16DstAddr Địa chỉ đích (Unicast hoặc Group 0xC000)
 * @param u8AlarmState 1: Báo động cháy, 0: Bình thường
 * @param s16Temp Nhiệt độ (ví dụ 45 độ C)
 * @param u16Smoke Nồng độ khói PPM
 */
int32_t app_mesh_SendFireAlarm(uint16_t u16DstAddr, uint8_t u8AlarmState, int16_t s16Temp, uint16_t u16Smoke);

/**
 * @brief Gui ban tin Vendor tho (Company ID 0x8888) toi Node hoac Group
 * @param u16Dst    Dia chi dich (unicast, group hoac 0xFFFF)
 * @param u8OpIdx   Chi so opcode 0x00..0x3F (byte tren khong = 0xC0 | u8OpIdx)
 * @param pu8Data   Tham so, toi da 8 byte (co the NULL neu u16Len = 0)
 * @param u16Len    So byte tham so
 */
int32_t app_mesh_SendVendorRaw(uint16_t u16Dst, uint8_t u8OpIdx, const uint8_t *pu8Data, uint16_t u16Len);

/** @brief Kiểm tra xem node đã được cấp mạng (Provisioned) hay chưa */
bool app_mesh_IsProvisioned(void);

/** @brief Lấy trạng thái báo cháy hiện tại của node (0 = Bình thường, 1 = Đang báo động) */
uint8_t app_mesh_GetAlarmState(void);

/** @brief Cập nhật trạng thái báo cháy của node */
void app_mesh_SetAlarmState(uint8_t u8AlarmState);

/** @brief In danh sách các thiết bị phát BLE Mesh đã quét được */
void app_mesh_PrintScannedDevices(void);

/** @brief Xóa danh sách thiết bị phát BLE Mesh đã quét */
void app_mesh_ClearScannedDevices(void);

/** @brief Lấy số lượng thiết bị phát BLE Mesh đã phát hiện */
uint32_t app_mesh_GetScannedDeviceCount(void);

/** @brief Khoi dong lai cau hinh AppKey/model cua node sau khi provisioned. */
int32_t app_mesh_GatewayJoin(void);

/** @brief Kiểm tra xem Gateway đã join mạng HOMI chưa */
bool app_mesh_GetGatewayJoined(void);

/** @brief Subscribe OnOff/Vendor models vao group address va luu cau hinh */
int32_t app_mesh_SetGroupAddress(uint16_t u16GroupAddr);
uint16_t app_mesh_GetGroupAddress(void);

/** @brief Gui Health Fault Get toi node */
int32_t app_mesh_HealthFaultGet(uint16_t u16Dst, uint16_t u16CompanyId);

/** @brief Gui Health Attention Set (0 = tat, >0 = so giay bat Attention) */
int32_t app_mesh_HealthAttentionSet(uint16_t u16Dst, uint8_t u8Attention);

/** @brief Gui Health Attention Get */
int32_t app_mesh_HealthAttentionGet(uint16_t u16Dst);


/* ================== GENERIC LEVEL CLIENT APIs ================== */
/**
 * @brief Gửi lệnh Generic Level Set tới Node
 * @param u16Dst Địa chỉ Unicast/Group đích
 * @param s16Level Giá trị level (-32768..32767)
 * @param bAck true: Có phản hồi, false: Unacknowledged
 */
int32_t app_mesh_SendGenericLevelSet(uint16_t u16Dst, int16_t s16Level, bool bAck);

/**
 * @brief Gửi lệnh Generic Level Get để truy vấn trạng thái Level hiện tại
 * @param u16Dst Địa chỉ Unicast/Group đích
 */
int32_t app_mesh_SendGenericLevelGet(uint16_t u16Dst);

int32_t app_mesh_SendLightnessGet(uint16_t u16Dst);
int32_t app_mesh_SendLightnessSet(uint16_t u16Dst, uint16_t u16Lightness, bool bAck);


int32_t app_mesh_SendOnOffGet(uint16_t u16Dst);
int32_t app_mesh_SendOnOffSet(uint16_t u16Dst, uint8_t u8OnOff, bool bAck);

/* ===== Light CTL Client APIs ===== */
int32_t app_mesh_SendCtlGet(uint16_t u16Dst);
int32_t app_mesh_SendCtlSet(uint16_t u16Dst, uint16_t lightness, uint16_t temperature, int16_t delta_uv, bool bAck);
int32_t app_mesh_SendCtlTempGet(uint16_t u16Dst);
int32_t app_mesh_SendCtlTempSet(uint16_t u16Dst, uint16_t temperature, int16_t delta_uv, bool bAck);

int32_t app_mesh_SendDefTransTimeGet(uint16_t u16Dst);
int32_t app_mesh_SendDefTransTimeSet(uint16_t u16Dst, uint32_t u32Ms, bool bAck);