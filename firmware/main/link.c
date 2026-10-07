// =============================================================================
//  link.c - access point, TCP server and transmit queue
// =============================================================================
//
//  Tasks (both on core 0):
//    link_rx - accepts the client and turns received bytes into lines,
//              which are executed as commands;
//    link_tx - drains the transmit stream buffer into the socket.
//
//  Senders never touch the socket. They only copy text into a stream buffer,
//  so even the control loop can report events without waiting on the
//  network.
// =============================================================================

#include "link.h"

#include <stdio.h>
#include <string.h>
#include "config.h"
#include "driver/usb_serial_jtag.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char *TAG = "link";

#define TX_BUFFER_SIZE      16384
#define RX_LINE_MAX         256

static link_line_handler_t s_on_line;
static link_event_handler_t s_on_connect;
static link_event_handler_t s_on_disconnect;

static StreamBufferHandle_t s_tx;
static SemaphoreHandle_t s_tx_lock;    // a stream buffer allows only one writer at a time
static volatile int s_client = -1;

bool link_connected(void)
{
    return s_client >= 0;
}

// Copies text into the transmit buffer. wait = 0 drops the text if it does
// not fit completely (never half a line).
static void enqueue(const char *text, TickType_t wait, bool mirror)
{
    // Mirror for bench tests, but only with a USB host attached: without one
    // the console would stall the caller (possibly the control loop).
    if (mirror && usb_serial_jtag_is_connected()) {
        fputs(text, stdout);
    }
    if (!link_connected()) {
        return;
    }
    const size_t len = strlen(text);
    if (xSemaphoreTake(s_tx_lock, wait) != pdTRUE) {
        return;
    }
    if (wait == 0 && xStreamBufferSpacesAvailable(s_tx) < len) {
        xSemaphoreGive(s_tx_lock);
        return;  // never send half a line
    }
    xStreamBufferSend(s_tx, text, len, wait);
    xSemaphoreGive(s_tx_lock);
}

void link_send(const char *text)
{
    enqueue(text, 0, true);
}

void link_send_blocking(const char *text)
{
    enqueue(text, pdMS_TO_TICKS(2000), true);
}

void link_send_quiet(const char *text)
{
    enqueue(text, 0, false);
}

// Sends whatever is queued. A failed send is ignored here: link_rx notices
// the dead socket and closes it.
static void tx_task(void *arg)
{
    uint8_t chunk[1024];
    for (;;) {
        const size_t len = xStreamBufferReceive(s_tx, chunk, sizeof(chunk), portMAX_DELAY);
        const int sock = s_client;
        size_t sent = 0;
        while (sock >= 0 && sent < len) {
            const int n = send(sock, chunk + sent, len - sent, 0);
            if (n <= 0) {
                break;  // the server task notices the dead socket
            }
            sent += n;
        }
    }
}

static void close_client(void)
{
    const int sock = s_client;
    if (sock < 0) {
        return;
    }
    s_client = -1;
    shutdown(sock, SHUT_RDWR);
    close(sock);
    ESP_LOGI(TAG, "client gone");
    // Deliberately no change to motors or driving state on link loss.
    s_on_disconnect();
}

static void accept_client(int listen_sock)
{
    struct sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);
    const int sock = accept(listen_sock, (struct sockaddr *)&addr, &addr_len);
    if (sock < 0) {
        return;
    }
    // A new phone connection replaces a stale one (e.g. after roaming).
    close_client();
    const int one = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    const struct timeval timeout = {.tv_sec = 1};
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    s_client = sock;
    ESP_LOGI(TAG, "client %s", inet_ntoa(addr.sin_addr));
    vTaskDelay(pdMS_TO_TICKS(LINK_HELLO_DELAY_MS));
    s_on_connect();
}

// Waits on the listening socket and the client at the same time (select), so
// a reconnecting phone is accepted even while an old socket hangs.
static void server_task(void *arg)
{
    const int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    const int one = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    const struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_GRUZIK_TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    bind(listen_sock, (const struct sockaddr *)&addr, sizeof(addr));
    listen(listen_sock, 2);

    char line[RX_LINE_MAX];
    size_t line_len = 0;
    char buf[256];
    for (;;) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(listen_sock, &fds);
        int max_fd = listen_sock;
        const int client = s_client;
        if (client >= 0) {
            FD_SET(client, &fds);
            max_fd = client > max_fd ? client : max_fd;
        }
        if (select(max_fd + 1, &fds, NULL, NULL, NULL) <= 0) {
            continue;
        }
        if (FD_ISSET(listen_sock, &fds)) {
            accept_client(listen_sock);
            line_len = 0;
            continue;
        }
        if (client < 0 || !FD_ISSET(client, &fds)) {
            continue;
        }
        const int n = recv(client, buf, sizeof(buf), 0);
        if (n <= 0) {
            close_client();
            line_len = 0;
            continue;
        }
        for (int i = 0; i < n; ++i) {
            const char c = buf[i];
            if (c == '\n') {
                line[line_len] = '\0';
                s_on_line(line);
                line_len = 0;
            } else if (c != '\r' && line_len < sizeof(line) - 1) {
                line[line_len++] = c;
            }
        }
    }
}

// NVS (Wi-Fi calibration data), network stack and the WPA2 access point.
// Power saving is off: latency matters more than current here.
static esp_err_t start_access_point(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop");
    esp_netif_create_default_wifi_ap();

    const wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_cfg), TAG, "wifi init");
    wifi_config_t ap_cfg = {
        .ap = {
            .ssid = CONFIG_GRUZIK_WIFI_SSID,
            .ssid_len = strlen(CONFIG_GRUZIK_WIFI_SSID),
            .password = CONFIG_GRUZIK_WIFI_PASSWORD,
            .channel = CONFIG_GRUZIK_WIFI_CHANNEL,
            .max_connection = 2,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg), TAG, "config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "start");
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "AP '%s' on 192.168.4.1:%d", CONFIG_GRUZIK_WIFI_SSID, CONFIG_GRUZIK_TCP_PORT);
    return ESP_OK;
}

esp_err_t link_init(link_line_handler_t on_line, link_event_handler_t on_connect,
                    link_event_handler_t on_disconnect)
{
    s_on_line = on_line;
    s_on_connect = on_connect;
    s_on_disconnect = on_disconnect;
    s_tx = xStreamBufferCreate(TX_BUFFER_SIZE, 1);
    s_tx_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_ERROR(start_access_point(), TAG, "access point");
    // Network work stays on core 0; core 1 belongs to the control loop.
    xTaskCreatePinnedToCore(server_task, "link_rx", 6144, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(tx_task, "link_tx", 4096, NULL, 5, NULL, 0);
    return ESP_OK;
}
