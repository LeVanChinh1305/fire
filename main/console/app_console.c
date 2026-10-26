#include "app_console.h"
#include "app_common.h"
#include "app_config.h"
#include "app_blemesh.h"

#include "esp_console.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PROMPT_STR                      "fire"
#define DF_CONSOLE_MAX_CMDLINE_LENGTH   128
#define DF_CONSOLE_TASK_STACK_SIZE      (4096)

/* Giả lập thông số cảm biến */
static int16_t  s_s16SimTemp  = 300;   /* 30.0 oC */
static uint16_t s_u16SimSmoke = 20;    /* 20 PPM */

/* ================== COMMAND HANDLERS ================== */

static int app_console_CmdVersion(int argc, char **argv)
{
    printf("\n=== THONG TIN PHAN MEM ===\n");
    printf("Ten Firmware : %s\n", DF_FIRMWARE_NAME);
    printf("Phien ban    : %s\n", app_common_GetVersion());
    printf("Target Chip  : %s\n", CONFIG_IDF_TARGET);
    printf("ESP-IDF Ver  : %s\n\n", esp_get_idf_version());
    return 0;
}

static int app_console_CmdRestart(int argc, char **argv)
{
    printf("Khoi dong lai chip...\n");
    app_common_DelayMs(200);
    esp_restart();
    return 0;
}

static int app_console_CmdHeap(int argc, char **argv)
{
    size_t sram_free  = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t sram_min   = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    printf("\n=== BO NHO HE THONG ===\n");
    printf("Internal SRAM Free : %u bytes (Min Free: %u bytes)\n", (unsigned int)sram_free, (unsigned int)sram_min);
    printf("External PSRAM Free: %u bytes\n\n", (unsigned int)psram_free);
    return 0;
}

static int app_console_CmdDebug(int argc, char **argv)
{
    if (argc < 2) {
        printf("Cu phap: debug <0|1>\n");
        return 1;
    }
    int iVal = atoi(argv[1]);
    app_common_SetDebug(iVal != 0);
    printf("Che do Debug da chuyen thanh: %s\n", g_bDebug ? "BAT (ON)" : "TAT (OFF)");
    return 0;
}

static int app_console_CmdNvsErase(int argc, char **argv)
{
    printf("\n>>> Tien hanh xoa sach phan vung NVS va khoi dong lai... <<<\n");
    nvs_flash_erase();
    app_common_DelayMs(300);
    esp_restart();
    return 0;
}

/* ================== BLE MESH COMMANDS ================== */

static int app_console_CmdMesh(int argc, char **argv)
{
    app_mesh_PrintStatus();
    return 0;
}

static int app_console_CmdMeshLog(int argc, char **argv)
{
    app_mesh_PrintLog();
    return 0;
}

static int app_console_CmdMeshReset(int argc, char **argv)
{
    return (app_mesh_Reset() == ERROR_CODE_OK) ? 0 : 1;
}

static int app_console_CmdMeshScan(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "clear") == 0) {
        app_mesh_ClearScannedDevices();
        printf("Da xoa toan bo danh sach thiet bi BLE Mesh da quet.\n");
        return 0;
    }
    app_mesh_PrintScannedDevices();
    return 0;
}

static int app_console_CmdHealth(int argc, char **argv)
{
    if (argc < 2) {
        printf("\n=== LENH HEALTH CLIENT ===\n");
        printf("  health get <addr_hex> [company_id]   : Health Fault Get (mac dinh CID=0x0000)\n");
        printf("  health att <addr_hex> <0|1..255>     : Attention Set (0=tat, >0=bat so giay)\n");
        printf("  health attget <addr_hex>             : Attention Get\n");
        printf("Vi du:\n");
        printf("  health get 0x0047\n");
        printf("  health get 0x0047 0x02E5\n");
        printf("  health att 0x0047 10\n");
        printf("  health att 0x0047 0\n\n");
        return 0;
    }

    if (strcmp(argv[1], "get") == 0) {
        if (argc < 3) {
            printf("Cu phap: health get <addr_hex> [company_id]\n");
            return 1;
        }
        uint16_t addr = (uint16_t)strtoul(argv[2], NULL, 0);
        uint16_t cid  = 0x0000;
        if (argc >= 4) {
            cid = (uint16_t)strtoul(argv[3], NULL, 0);
        }
        int32_t ret = app_mesh_HealthFaultGet(addr, cid);
        return (ret == ERROR_CODE_OK) ? 0 : 1;
    }

    if (strcmp(argv[1], "att") == 0) {
        if (argc < 4) {
            printf("Cu phap: health att <addr_hex> <0|1..255>\n");
            return 1;
        }
        uint16_t addr = (uint16_t)strtoul(argv[2], NULL, 0);
        uint8_t  att  = (uint8_t)atoi(argv[3]);
        int32_t ret = app_mesh_HealthAttentionSet(addr, att);
        return (ret == ERROR_CODE_OK) ? 0 : 1;
    }

    if (strcmp(argv[1], "attget") == 0) {
        if (argc < 3) {
            printf("Cu phap: health attget <addr_hex>\n");
            return 1;
        }
        uint16_t addr = (uint16_t)strtoul(argv[2], NULL, 0);
        int32_t ret = app_mesh_HealthAttentionGet(addr);
        return (ret == ERROR_CODE_OK) ? 0 : 1;
    }

    printf("Sub-command khong hop le. Go 'health' de xem huong dan.\n");
    return 1;
}

/* ================== HOMI GATEWAY COMMANDS ================== */

static int app_console_CmdMeshGw(int argc, char **argv)
{
    if (argc < 2) {
        printf("\n=== CAC LENH HOMI GATEWAY ===\n");
        printf("  meshgw status          : Xem provisioned, AppKey/model binding va Group\n");
        printf("  meshgw join            : Bat lai cau hinh local (node phai duoc provision truoc)\n");
        printf("  meshgw sub [group_hex] : Subscribe OnOff/Vendor vao Group (mac dinh 0xC000)\n\n");
        return 0;
    }

    if (strcmp(argv[1], "status") == 0) {
        printf("\n=== TRANG THAI HOMI GATEWAY ===\n");
        printf("Gateway joined  : %s\n",
               app_mesh_GetGatewayJoined() ? "HOAN TAT" : "DANG CHO / CO LOI");
        printf("Node provisioned: %s\n",
               app_mesh_IsProvisioned() ? "DA PROVISIONED" : "CHUA PROVISIONED");
        printf("Group address   : 0x%04X\n\n", app_mesh_GetGroupAddress());
        app_mesh_PrintStatus();
        return 0;
    }

    if (strcmp(argv[1], "join") == 0) {
        printf("\n>>> Dang khoi tao lai cau hinh HOMI local...\n");
        int32_t ret = app_mesh_GatewayJoin();
        if (ret == ERROR_CODE_OK || ret == ERROR_CODE_BUSY) {
            printf(">>> Da tiep nhan yeu cau. Kiem tra 'meshgw status' sau vai giay.\n\n");
        } else {
            printf(">>> Khong the cau hinh. Node phai duoc provision vao HOMI truoc; xem log.\n\n");
        }
        return (ret == ERROR_CODE_OK || ret == ERROR_CODE_BUSY) ? 0 : 1;
    }

    if (strcmp(argv[1], "sub") == 0) {
        uint16_t u16Group = app_mesh_GetGroupAddress();
        if (argc >= 3) {
            char *pcEnd = NULL;
            unsigned long ulAddress = strtoul(argv[2], &pcEnd, 0);
            if (pcEnd == argv[2] || *pcEnd != '\0' || ulAddress > 0xFFFF) {
                printf("Group address khong hop le. Vi du: meshgw sub 0xC000\n");
                return 1;
            }
            u16Group = (uint16_t)ulAddress;
        }
        int32_t ret = app_mesh_SetGroupAddress(u16Group);
        if (ret == ERROR_CODE_OK) {
            printf("Da yeu cau subscribe cac model vao Group 0x%04X; kiem tra status/log.\n", u16Group);
            return 0;
        }
        printf("Subscribe Group 0x%04X that bai; xem log.\n", u16Group);
        return 1;
    }

    printf("Sub-command khong hop le. Go 'meshgw' de xem huong dan.\n");
    return 1;
}

static int app_console_CmdMeshSend(int argc, char **argv)
{
    if (argc < 3) {
        printf("\nCu phap: meshsend <dia_chi_hex> <0|1>\n");
        printf("Vi du  : meshsend 0x0002 1   (Bat OnOff Node 0x0002)\n");
        printf("         meshsend 0xC000 1   (Bat OnOff ca Group 0xC000)\n");
        printf("         meshsend 0x0002 0   (Tat OnOff Node 0x0002)\n\n");
        return 1;
    }

    uint16_t u16Dst = (uint16_t)strtol(argv[1], NULL, 0);
    uint8_t  u8OnOff = (uint8_t)atoi(argv[2]);

    int32_t ret = app_mesh_SendOnOff(u16Dst, u8OnOff);
    return (ret == ERROR_CODE_OK) ? 0 : 1;
}

/* ================== FIRE ALARM COMMANDS ================== */

static int app_console_CmdFire(int argc, char **argv)
{
    if (argc < 2) {
        printf("\n=== CAC LENH TEST BAO CHAY (FIRE) ===\n");
        printf("  fire status                 : Xem trang thai cam bien va canh bao\n");
        printf("  fire alarm <0|1>            : Kich hoat (1) hoac Tat (0) canh bao chay tai cho va tren mang\n");
        printf("  fire test [dst_hex]         : Gia lap su co chay va ban canh bao Mesh (Mac dinh den 0xC000)\n");
        printf("  fire sensor <temp_oC> <ppm> : Gia lap chi so cam bien nhiet do va khoi\n\n");
        return 1;
    }

    if (strcmp(argv[1], "status") == 0) {
        uint8_t u8Alarm = app_mesh_GetAlarmState();
        printf("\n=== TRANG THAI CAM BIEN BAO CHAY ===\n");
        printf("Trang thai canh bao : %s\n", u8Alarm ? "DANG CO CHAY (ALARM ACTIVE)" : "AN TOAN (NORMAL)");
        printf("Nhiet do hien tai   : %.1f oC\n", (float)s_s16SimTemp / 10.0f);
        printf("Nong do khoi        : %u PPM\n", s_u16SimSmoke);
        printf("Mesh Provisioned    : %s\n\n", app_mesh_IsProvisioned() ? "DA VAO MANG" : "CHUA VAO MANG");
        return 0;
    }

    if (strcmp(argv[1], "alarm") == 0) {
        if (argc < 3) {
            printf("Cu phap: fire alarm <0|1>\n");
            return 1;
        }
        uint8_t u8Alarm = (uint8_t)atoi(argv[2]);
        app_mesh_SetAlarmState(u8Alarm);

        uint16_t u16Dst = DF_MESH_GROUP_FIRE_ALARM; /* 0xC000 */
        if (argc >= 4) {
            u16Dst = (uint16_t)strtol(argv[3], NULL, 0);
        }

        if (u8Alarm) {
            s_s16SimTemp  = 650;  /* 65 oC */
            s_u16SimSmoke = 650;  /* 650 PPM */
            printf("\n>>> KICH HOAT BAO DONG CHAY! Phat canh bao toi Mesh Group 0x%04X... <<<\n", u16Dst);
        } else {
            s_s16SimTemp  = 295;  /* 29.5 oC */
            s_u16SimSmoke = 15;   /* 15 PPM */
            printf("\n>>> DA TAT BAO DONG CHAY. Phat tin nhan an toan toi Mesh Group 0x%04X... <<<\n", u16Dst);
        }

        app_mesh_SendFireAlarm(u16Dst, u8Alarm, s_s16SimTemp, s_u16SimSmoke);
        return 0;
    }

    if (strcmp(argv[1], "test") == 0) {
        uint16_t u16Dst = DF_MESH_GROUP_FIRE_ALARM;
        if (argc >= 3) {
            u16Dst = (uint16_t)strtol(argv[2], NULL, 0);
        }

        printf("\n=======================================================\n");
        printf(">>> GIA LAP SU CO CHAY KHAN CAP <<<\n");
        printf("Nhiet do vuot nguong: 72.5 oC | Nong do khoi: 950 PPM\n");
        printf("Phat ban tin canh bao khan cap toi Mesh 0x%04X...\n", u16Dst);
        printf("=======================================================\n");

        s_s16SimTemp  = 725;
        s_u16SimSmoke = 950;
        app_mesh_SetAlarmState(1);
        app_mesh_SendFireAlarm(u16Dst, 1, s_s16SimTemp, s_u16SimSmoke);
        return 0;
    }

    if (strcmp(argv[1], "sensor") == 0) {
        if (argc < 4) {
            printf("Cu phap: fire sensor <nhiet_do_oC> <nong_do_khoi_ppm>\n");
            printf("Vi du  : fire sensor 45.5 250\n");
            return 1;
        }
        float fTemp = atof(argv[2]);
        s_s16SimTemp  = (int16_t)(fTemp * 10.0f);
        s_u16SimSmoke = (uint16_t)atoi(argv[3]);
        printf("Da cap nhat cam bien gia lap: %.1f oC | %u PPM\n", fTemp, s_u16SimSmoke);
        return 0;
    }

    printf("Lenh khong hop le. Go 'fire' de xem huong dan.\n");
    return 1;
}
static int app_console_CmdVndSend(int argc, char **argv)
{
    if (argc < 3) {
        printf("\nCu phap: vndsend <dst_hex> <op_hex 00..3F> [data_hex toi da 8 byte]\n");
        printf("Vi du  : vndsend 0xFFFF 11 020294C960FAE838\n");
        printf("(op 11 = byte D1 tren khong; CHI dung tren thiet bi trong lab)\n\n");
        return 1;
    }
 
    char *pcEnd = NULL;
    unsigned long ulDst = strtoul(argv[1], &pcEnd, 0);
    if (pcEnd == argv[1] || *pcEnd != '\0' || ulDst > 0xFFFF) {
        printf("Dia chi dich khong hop le\n");
        return 1;
    }
    unsigned long ulOp = strtoul(argv[2], &pcEnd, 16);
    if (pcEnd == argv[2] || *pcEnd != '\0' || ulOp > 0x3F) {
        printf("Op phai la hex trong khoang 00..3F\n");
        return 1;
    }
 
    uint8_t  au8Data[8];
    uint16_t u16Len = 0;
    if (argc >= 4) {
        size_t n = strlen(argv[3]);
        if ((n % 2) != 0 || (n / 2) > sizeof(au8Data)) {
            printf("data_hex phai la cac cap ky tu hex, toi da 8 byte\n");
            return 1;
        }
        for (size_t i = 0; i < n; i += 2) {
            char acB[3] = { argv[3][i], argv[3][i + 1], 0 };
            unsigned long ulV = strtoul(acB, &pcEnd, 16);
            if (*pcEnd != '\0') {
                printf("data_hex chua ky tu khong phai hex\n");
                return 1;
            }
            au8Data[u16Len++] = (uint8_t)ulV;
        }
    }
 
    int32_t ret = app_mesh_SendVendorRaw((uint16_t)ulDst, (uint8_t)ulOp, au8Data, u16Len);
    return (ret == ERROR_CODE_OK) ? 0 : 1;
}

static int app_console_CmdLevel(int argc, char **argv)
{
    if (argc < 3) {
        printf("\nCu phap: level <get|set> <addr_hex> [val_s16] [ack=0|1]\n");
        printf("Vi du  : level get 0x0047\n");
        printf("         level set 0x0047 1000 1\n\n");
        return 1;
    }

    uint16_t addr = (uint16_t)strtoul(argv[2], NULL, 0);

    if (strcmp(argv[1], "get") == 0) {
        return (app_mesh_SendGenericLevelGet(addr) == ERROR_CODE_OK) ? 0 : 1;
    }
    if (strcmp(argv[1], "set") == 0) {
        if (argc < 4) return 1;
        int16_t val = (int16_t)atoi(argv[3]);
        bool bAck = (argc >= 5) ? (atoi(argv[4]) != 0) : true;
        return (app_mesh_SendGenericLevelSet(addr, val, bAck) == ERROR_CODE_OK) ? 0 : 1;
    }
    return 1;
}

static int app_console_CmdLightness(int argc, char **argv)
{
    if (argc < 3) {
        printf("\nCu phap: lightness <get|set> <addr_hex> [val_u16] [ack=0|1]\n");
        printf("Vi du  : lightness get 0x0043\n");
        printf("         lightness set 0x0043 25000 1\n\n");
        return 1;
    }

    uint16_t addr = (uint16_t)strtoul(argv[2], NULL, 0);

    if (strcmp(argv[1], "get") == 0) {
        return (app_mesh_SendLightnessGet(addr) == ERROR_CODE_OK) ? 0 : 1;
    }
    if (strcmp(argv[1], "set") == 0) {
        if (argc < 4) return 1;
        uint16_t val = (uint16_t)atoi(argv[3]);
        bool bAck = (argc >= 5) ? (atoi(argv[4]) != 0) : true;
        return (app_mesh_SendLightnessSet(addr, val, bAck) == ERROR_CODE_OK) ? 0 : 1;
    }
    return 1;
}

static int app_console_CmdCtl(int argc, char **argv)
{
    if (argc < 3) {
        printf("\nCu phap:\n");
        printf("  ctl get      <addr_hex>\n");
        printf("  ctl set      <addr_hex> <lightness> <temp> [delta_uv] [ack]\n");
        printf("  ctl tempget  <addr_hex>\n");
        printf("  ctl tempset  <addr_hex> <temp> [delta_uv] [ack]\n");
        printf("Vi du:\n");
        printf("  ctl get 0x0043\n");
        printf("  ctl set 0x0043 30000 4000 0 1\n");
        printf("  ctl tempset 0x0044 3500 0 1\n\n");
        return 1;
    }

    uint16_t addr = (uint16_t)strtoul(argv[2], NULL, 0);

    if (strcmp(argv[1], "get") == 0) {
        return (app_mesh_SendCtlGet(addr) == ERROR_CODE_OK) ? 0 : 1;
    }
    if (strcmp(argv[1], "set") == 0) {
        if (argc < 5) return 1;
        uint16_t light = (uint16_t)atoi(argv[3]);
        uint16_t temp  = (uint16_t)atoi(argv[4]);
        int16_t  duv   = (argc >= 6) ? (int16_t)atoi(argv[5]) : 0;
        bool     bAck  = (argc >= 7) ? (atoi(argv[6]) != 0) : true;
        return (app_mesh_SendCtlSet(addr, light, temp, duv, bAck) == ERROR_CODE_OK) ? 0 : 1;
    }
    if (strcmp(argv[1], "tempget") == 0) {
        return (app_mesh_SendCtlTempGet(addr) == ERROR_CODE_OK) ? 0 : 1;
    }
    if (strcmp(argv[1], "tempset") == 0) {
        if (argc < 4) return 1;
        uint16_t temp = (uint16_t)atoi(argv[3]);
        int16_t  duv  = (argc >= 5) ? (int16_t)atoi(argv[4]) : 0;
        bool     bAck = (argc >= 6) ? (atoi(argv[5]) != 0) : true;
        return (app_mesh_SendCtlTempSet(addr, temp, duv, bAck) == ERROR_CODE_OK) ? 0 : 1;
    }
    return 1;
}


static int app_console_CmdDefTt(int argc, char **argv)
{
    if (argc < 3) {
        printf("\nCu phap: dtt <get|set> <addr_hex> [ms] [ack=0|1]\n");
        printf("Vi du  : dtt get 0x0043\n");
        printf("         dtt set 0x0043 2000 1\n\n");
        return 1;
    }
    uint16_t addr = (uint16_t)strtoul(argv[2], NULL, 0);

    if (strcmp(argv[1], "get") == 0) {
        return (app_mesh_SendDefTransTimeGet(addr) == ERROR_CODE_OK) ? 0 : 1;
    }
    if (strcmp(argv[1], "set") == 0) {
        if (argc < 4) return 1;
        uint32_t ms = (uint32_t)strtoul(argv[3], NULL, 0);
        bool bAck = (argc >= 5) ? (atoi(argv[4]) != 0) : true;
        return (app_mesh_SendDefTransTimeSet(addr, ms, bAck) == ERROR_CODE_OK) ? 0 : 1;
    }
    return 1;
}

/* ================== ĐĂNG KÝ DANH SÁCH LỆNH ================== */

static void app_console_RegisterCommands(void)
{
    const esp_console_cmd_t stCmdVer = {
        .command = "version",
        .help    = "Xem ten firmware va phien ban ESP-IDF",
        .hint    = NULL,
        .func    = &app_console_CmdVersion,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdVer));

    const esp_console_cmd_t stCmdRe = {
        .command = "restart",
        .help    = "Khoi dong lai ESP32",
        .hint    = NULL,
        .func    = &app_console_CmdRestart,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdRe));

    const esp_console_cmd_t stCmdHeap = {
        .command = "heap",
        .help    = "Kiem tra dung luong RAM/PSRAM con trong",
        .hint    = NULL,
        .func    = &app_console_CmdHeap,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdHeap));

    const esp_console_cmd_t stCmdDbg = {
        .command = "debug",
        .help    = "Bat/tat log debug (debug <0|1>)",
        .hint    = NULL,
        .func    = &app_console_CmdDebug,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdDbg));

    const esp_console_cmd_t stCmdNvs = {
        .command = "nvserase",
        .help    = "Xoa sach NVS Flash (xoa sach role cu/thong tin mang) va restart",
        .hint    = NULL,
        .func    = &app_console_CmdNvsErase,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdNvs));

    /* BLE Mesh Commands */
    const esp_console_cmd_t stCmdMesh = {
        .command = "mesh",
        .help    = "In trang thai chi tiet BLE Mesh Node / HOMI Gateway",
        .hint    = NULL,
        .func    = &app_console_CmdMesh,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdMesh));

    const esp_console_cmd_t stCmdMeshLog = {
        .command = "meshlog",
        .help    = "In lich su cac ban tin mesh gan nhat",
        .hint    = NULL,
        .func    = &app_console_CmdMeshLog,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdMeshLog));

    const esp_console_cmd_t stCmdMeshReset = {
        .command = "meshreset",
        .help    = "Xoa trang thai cap mang (reset ve unprovisioned) va restart",
        .hint    = NULL,
        .func    = &app_console_CmdMeshReset,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdMeshReset));

    const esp_console_cmd_t stCmdMeshSend = {
        .command = "meshsend",
        .help    = "Gui Generic OnOff: meshsend <dst_hex> <0|1>",
        .hint    = NULL,
        .func    = &app_console_CmdMeshSend,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdMeshSend));

    const esp_console_cmd_t stCmdMeshScan = {
        .command = "meshscan",
        .help    = "Xem danh sach cac thiet bi BLE Mesh dang quet duoc (cu phap: meshscan [clear])",
        .hint    = NULL,
        .func    = &app_console_CmdMeshScan,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdMeshScan));

    /* HOMI Gateway Commands */
    const esp_console_cmd_t stCmdMeshGw = {
        .command = "meshgw",
        .help    = "Quan ly HOMI Gateway: meshgw [status|join|sub [group_hex]]",
        .hint    = NULL,
        .func    = &app_console_CmdMeshGw,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdMeshGw));

    /* Fire Alarm Commands */
    const esp_console_cmd_t stCmdFire = {
        .command = "fire",
        .help    = "Cac lenh kiem tra va gia lap bao chay (status/alarm/test/sensor)",
        .hint    = NULL,
        .func    = &app_console_CmdFire,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdFire));

    /* Vendor raw send (lab) */
    const esp_console_cmd_t stCmdVndSend = {
        .command = "vndsend",
        .help    = "Gui vendor tho (lab): vndsend <dst_hex> <op_hex> [data_hex]",
        .hint    = NULL,
        .func    = &app_console_CmdVndSend,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdVndSend));

    const esp_console_cmd_t stCmdHealth = {
        .command = "health",
        .help    = "Health Client: health [get|att|attget] ...",
        .hint    = NULL,
        .func    = &app_console_CmdHealth,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdHealth));

    const esp_console_cmd_t stCmdLevel = {
        .command = "level",
        .help    = "Generic Level: level <get|set> <addr_hex> [val] [ack]",
        .hint    = NULL,
        .func    = &app_console_CmdLevel,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdLevel));

    const esp_console_cmd_t stCmdLightness = {
        .command = "lightness",
        .help    = "Light Lightness Client: lightness <get|set> <addr_hex> [val] [ack]",
        .hint    = NULL,
        .func    = &app_console_CmdLightness,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdLightness));

    const esp_console_cmd_t stCmdCtl = {
        .command = "ctl",
        .help    = "Light CTL Client: ctl <get|set|tempget|tempset> ...",
        .hint    = NULL,
        .func    = &app_console_CmdCtl,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdCtl));

    const esp_console_cmd_t stCmdDtt = {
        .command = "dtt",
        .help    = "Default Transition Time: dtt <get|set> <addr_hex> [ms] [ack]",
        .hint    = NULL,
        .func    = &app_console_CmdDefTt,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&stCmdDtt));

}

/* ================== KHỞI TẠO CONSOLE REPL ================== */

void app_console_Init(void)
{
    esp_console_repl_t *pRepl = NULL;
    esp_console_repl_config_t stReplConfig = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

    stReplConfig.prompt = PROMPT_STR "> ";
    stReplConfig.max_cmdline_length = DF_CONSOLE_MAX_CMDLINE_LENGTH;
    stReplConfig.task_stack_size = DF_CONSOLE_TASK_STACK_SIZE;

    esp_console_register_help_command();
    app_console_RegisterCommands();

#if CONFIG_ESP_CONSOLE_UART
    esp_console_dev_uart_config_t stHwConfig = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&stHwConfig, &stReplConfig, &pRepl));
#elif CONFIG_ESP_CONSOLE_USB_CDC
    esp_console_dev_usb_cdc_config_t stHwConfig = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&stHwConfig, &stReplConfig, &pRepl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t stHwConfig = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&stHwConfig, &stReplConfig, &pRepl));
#else
    /* Mac dinh khoi tao UART */
    esp_console_dev_uart_config_t stHwConfig = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&stHwConfig, &stReplConfig, &pRepl));
#endif

    printf("\n");
    printf("==================================================\n");
    printf("   %s Console CLI v%s\n", DF_FIRMWARE_NAME, app_common_GetVersion());
    printf("   Go 'help' de xem tat ca cac lenh ho tro\n");
    printf("==================================================\n\n");

    ESP_ERROR_CHECK(esp_console_start_repl(pRepl));
}

