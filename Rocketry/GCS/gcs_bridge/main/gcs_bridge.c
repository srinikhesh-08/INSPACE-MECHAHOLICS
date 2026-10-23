#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sx1276.h"

#define MAIN_HZ      865500000u
#define BACKUP_HZ    867250000u
#define LORA_SF      7              /* your pad's spreading factor */
#define SYNC_WORD    0x47           /* 0x40 + team 7 */
#define GCS_PWR_DBM  14             /* power for commands up to the rocket */

static void tune(const char *name, uint32_t hz)
{
    lora_configure(hz, LORA_SF, SYNC_WORD);
    lora_set_power_dbm(GCS_PWR_DBM);
    lora_start_rx();
    printf("INFO|listening on %s %lu Hz\n", name, (unsigned long)hz);
}

/* Lines from the laptop:   CH MAIN | CH BACKUP | CMD <hex bytes>   e.g.  CMD AA 07 00 01 00 AC */
static void handle_line(char *line)
{
    if (strcmp(line, "CH MAIN") == 0)   { tune("MAIN", MAIN_HZ);     return; }
    if (strcmp(line, "CH BACKUP") == 0) { tune("BACKUP", BACKUP_HZ); return; }

    if (strncmp(line, "CMD ", 4) == 0) {
        uint8_t frame[16];
        int n = 0;
        for (char *tok = strtok(line + 4, " ");
             tok != NULL && n < (int)sizeof(frame);
             tok = strtok(NULL, " ")) {
            frame[n++] = (uint8_t)strtol(tok, NULL, 16);
        }
        if (n > 0 && lora_send(frame, (uint8_t)n) == ESP_OK) {
            printf("INFO|sent %d bytes\n", n);
        } else {
            printf("ERR|send failed\n");
        }
        lora_start_rx();                       /* back to listening */
        return;
    }
    printf("ERR|unknown line: %s\n", line);
}

void app_main(void)
{
    if (lora_init() != ESP_OK) {
        printf("ERR|SX1276 not found, check wiring\n");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    tune("MAIN", MAIN_HZ);
    printf("INFO|GCS bridge ready\n");

    char line[96];
    int len = 0;
    uint8_t rx[255];

    while (true) {
        /* radio -> laptop: one line per packet, "RX|rssi|snr|payload" */
        int n = lora_poll_rx(rx, sizeof(rx));
        if (n > 0) {
            printf("RX|%d|%.1f|", lora_last_rssi(), lora_last_snr());
            fwrite(rx, 1, (size_t)n, stdout);
            if (rx[n - 1] != '\n') printf("\n");
            fflush(stdout);
        }

        /* laptop -> radio: collect characters until the end of a line (same getchar idea as your debug console) */
        int c;
        while ((c = getchar()) != EOF) {
            if (c == '\n' || c == '\r') {
                if (len > 0) { line[len] = '\0'; handle_line(line); len = 0; }
            } else if (len < (int)sizeof(line) - 1) {
                line[len++] = (char)c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}