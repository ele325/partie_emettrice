#include "sleep_manager.h"
#include "esp_sleep.h"
#include "esp_log.h"

static const char *TAG = "SLEEP";

void sleep_manager_configure_timer(uint64_t sleep_us)
{
    esp_sleep_enable_timer_wakeup(sleep_us);
    ESP_LOGI(TAG, "Deep sleep configuré pour %"PRIu64" µs", sleep_us);
}

void sleep_manager_enter_deep_sleep(void)
{
    ESP_LOGI(TAG, "Entrée en deep sleep...");
    esp_deep_sleep_start();
    // Ne retourne jamais
}