#include "app_common.h"

bool g_bDebug = true;

void app_common_DelayMs(uint32_t u32Ms)
{
    vTaskDelay(pdMS_TO_TICKS(u32Ms));
}

const char *app_common_GetVersion(void)
{
    return DF_FIRMWARE_VERSION;
}

void app_common_SetDebug(bool bEnable)
{
    g_bDebug = bEnable;
}

bool app_common_Lock(SemaphoreHandle_t hMutex, const char *pcName, uint32_t u32TimeoutMs)
{
    if (hMutex == NULL) {
        LREP_ERROR("LOCK", "Mutex NULL: %s", pcName ? pcName : "?");
        return false;
    }
    if (xSemaphoreTake(hMutex, pdMS_TO_TICKS(u32TimeoutMs)) != pdTRUE) {
        LREP_ERROR("LOCK", "Take mutex TIMEOUT (%ums): %s", (unsigned)u32TimeoutMs, pcName ? pcName : "?");
        return false;
    }
    LREP_DEBUG("LOCK", "[MUTEX TAKE] Task '%s' da lay: %s", pcTaskGetName(NULL), pcName ? pcName : "?");
    return true;
}

void app_common_Unlock(SemaphoreHandle_t hMutex, const char *pcName)
{
    if (hMutex != NULL) {
        LREP_DEBUG("LOCK", "[MUTEX RELEASE] Task '%s' da tra: %s", pcTaskGetName(NULL), pcName ? pcName : "?");
        xSemaphoreGive(hMutex);
    }
}
