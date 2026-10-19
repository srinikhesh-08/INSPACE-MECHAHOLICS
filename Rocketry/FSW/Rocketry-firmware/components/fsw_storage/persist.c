#include "persist.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "persist";
#define NS_NAME    "fsw"
#define KEY_REC    "rec"
#define REC_MAGIC  0xF5A15EC0u

static bool s_ready = false;

esp_err_t persist_init(void)
{
    esp_err_t e = nvs_flash_init();
    s_ready = (e == ESP_OK);
    if (!s_ready) {
        ESP_LOGE(TAG, "NVS init failed: %s (not erasing)", esp_err_to_name(e));
    }
    return e;
}

/* GROUND ONLY: wipes the saved record and re-creates the NVS area. */
esp_err_t persist_repair(void)
{
    ESP_LOGW(TAG, "repairing NVS on the pad");
    esp_err_t e = nvs_flash_erase();
    if (e == ESP_OK) e = nvs_flash_init();
    s_ready = (e == ESP_OK);
    return e;
}

bool persist_ready(void)
{
    return s_ready;
}

esp_err_t persist_save(const persist_record_t *rec)
{
    persist_record_t r = *rec;
    r.magic = REC_MAGIC;
    nvs_handle_t h;
    esp_err_t e = nvs_open(NS_NAME, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    e = nvs_set_blob(h, KEY_REC, &r, sizeof(r));
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    if (e != ESP_OK) ESP_LOGE(TAG, "save failed: %s", esp_err_to_name(e));
    return e;
}

bool persist_load(persist_record_t *out)
{
    nvs_handle_t h;
    if (nvs_open(NS_NAME, NVS_READONLY, &h) != ESP_OK) return false;
    persist_record_t r;
    size_t len = sizeof(r);
    esp_err_t e = nvs_get_blob(h, KEY_REC, &r, &len);
    nvs_close(h);
    if (e != ESP_OK || len != sizeof(r) || r.magic != REC_MAGIC) return false;
    *out = r;
    return true;
}

esp_err_t persist_clear(void)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NS_NAME, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    e = nvs_erase_key(h, KEY_REC);
    if (e == ESP_ERR_NVS_NOT_FOUND) e = ESP_OK;
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}