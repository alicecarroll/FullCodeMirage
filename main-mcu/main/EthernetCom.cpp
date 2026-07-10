//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// 100% Calude.................
// But I removed some stuff.
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

/*
 * Ethernet_com.cpp
 *
 * Driver for the WIZ850io Ethernet module (W5500 chip).
 * Uses WIZnet's ioLibrary_Driver directly so that the W5500's own
 * hardwired TCP/IP stack and internal 32KB TX/RX buffer are used,
 * rather than routing through ESP-IDF's lwIP stack.
 *
 * Does NOT initialise the SPI bus — that is done once in Initialize.cpp.
 * CS pin: IO10 (CS0)
 * RST pin: IO8  (RST0)
 *
 * ioLibrary_Driver dependency:
 *   Clone https://github.com/Wiznet/ioLibrary_Driver into your components/
 *   directory and add it to CMakeLists.txt REQUIRES.
 *   Required files from ioLibrary:
 *     Ethernet/W5500/w5500.c
 *     Ethernet/wizchip_conf.c
 *     Internet/DHCP/dhcp.c          (optional)
 *     Socket/socket.c
 *
 * Copyright notice for ioLibrary_Driver (retained as required):
 *   Copyright (c) 2013, WIZnet Co., LTD.
 *   All rights reserved.
 *   SPDX-License-Identifier: BSD-3-Clause
 */



// ioLibrary headers (from WIZnet ioLibrary_Driver component)
#include "EthernetCom.h"

#include "wizchip_conf.h"
#include "wizsocket.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Settings.h"

static const char *TAG = "Ethernet WIZ850io";

// SPI device handle — registered on the existing SPI2 bus
// static spi_device_handle_t WIZ_handle = NULL;
//^used to be s_spi, changed to fit settings.h

// ---------------------------------------------------------------------------
// ICMP ping helpers
// ---------------------------------------------------------------------------

// ICMP packet structure
typedef struct
{
    uint8_t type; // 8 = Echo Request, 0 = Echo Reply, use only 8 atm
    uint8_t code; // Always 0
    uint16_t checksum;
    uint16_t id;
    uint16_t seq;
    uint8_t data[32]; // Payload — 32 bytes is conventional ping size
} icmp_packet_t;

// Standard internet checksum (ones-complement sum of 16-bit words)
static uint16_t icmp_checksum(uint8_t *buf, size_t len)
{
    uint32_t sum = 0;
    while (len > 1)
    {
        sum += (buf[0] << 8) | buf[1];
        buf += 2;
        len -= 2;
    }
    if (len == 1)
        sum += (*buf << 8); // Odd byte
    while (sum >> 16)
        sum = (sum & 0xFFFF) + (sum >> 16); // Fold carry
    return (uint16_t)(~sum);
}

// Build a ready-to-send ICMP Echo Request packet
static void build_icmp_request(icmp_packet_t *pkt, uint16_t id, uint16_t seq, const char *message)
{
    pkt->type = 8; // Echo Request
    pkt->code = 0;
    pkt->checksum = 0; // Zero before computing
    pkt->id = htons(id);
    pkt->seq = htons(seq);

    memset(pkt->data, 0, sizeof(pkt->data));
    if (message)
        strncpy((char *)pkt->data, message, sizeof(pkt->data) - 1);

    // Compute checksum over the whole packet
    pkt->checksum = htons(icmp_checksum((uint8_t *)pkt, sizeof(icmp_packet_t)));
}

// ---------------------------------------------------------------------------
// ioLibrary SPI callback implementations
// These are the six functions ioLibrary needs to talk to the hardware.
// ---------------------------------------------------------------------------

static void wiz_cs_select(void)
{
    gpio_set_level((gpio_num_t)CS_WIZ_PIN, 0); // CS low = selected
    ESP_LOGD(TAG, "CS asserted on pin %d", CS_WIZ_PIN);
}

static void wiz_cs_deselect(void)
{
    gpio_set_level((gpio_num_t)CS_WIZ_PIN, 1); // CS high = deselected
    ESP_LOGD(TAG, "CS deasserted on pin %d", CS_WIZ_PIN);
}

static uint8_t wiz_spi_read_byte(void)
{
    spi_transaction_t t = {};
    t.length = 8; // 8 bits
    t.flags = SPI_TRANS_USE_RXDATA;
    esp_err_t err = spi_device_transmit(WIZ_handle, &t);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI read transaction failed: %s", esp_err_to_name(err));
    }
    ESP_LOGD(TAG, "SPI read byte returned: 0x%02X", t.rx_data[0]);
    return t.rx_data[0];
}

static void wiz_spi_write_byte(uint8_t byte)
{
    spi_transaction_t t = {};
    t.length = 8;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = byte;
    esp_err_t err = spi_device_transmit(WIZ_handle, &t);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI write transaction failed: %s", esp_err_to_name(err));
    }
}

// Burst read — much faster than byte-by-byte for large buffer reads
static void wiz_spi_read_burst(uint8_t *buf, uint16_t len)
{
    spi_transaction_t t = {};
    t.length = len * 8;
    t.rx_buffer = buf;
    t.tx_buffer = NULL; // ESP-IDF will clock out 0x00 bytes on MOSI
    spi_device_transmit(WIZ_handle, &t);
}

// Burst write — used when pushing data into W5500's TX buffer
static void wiz_spi_write_burst(uint8_t *buf, uint16_t len)
{
    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = buf;
    t.rx_buffer = NULL; // Explicitly discard anything clocked in on MISO
    spi_device_transmit(WIZ_handle, &t);
}

// ---------------------------------------------------------------------------
// Hardware reset
// ---------------------------------------------------------------------------

static void wiz_hw_reset(void)
{
    gpio_set_level((gpio_num_t)Reset_WIZ_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10)); // Hold reset for 10ms
    gpio_set_level((gpio_num_t)Reset_WIZ_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(150)); // Wait for W5500 to come up (datasheet: min 50ms)
}

// ---------------------------------------------------------------------------
// Application-facing functions
// ---------------------------------------------------------------------------

esp_err_t wiz_init(void)
{
    if (WIZ_handle == NULL)
    {
        ESP_LOGE(TAG, "W5500 SPI device is not registered");
        return ESP_ERR_INVALID_STATE;
    }

    wiz_hw_reset();

    reg_wizchip_cs_cbfunc(wiz_cs_select, wiz_cs_deselect);
    reg_wizchip_spi_cbfunc(wiz_spi_read_byte, wiz_spi_write_byte);
    reg_wizchip_spiburst_cbfunc(wiz_spi_read_burst, wiz_spi_write_burst);

    uint8_t tx_size[_WIZCHIP_SOCK_NUM_] = {8, 8, 0, 0, 0, 0, 0, 0};
    uint8_t rx_size[_WIZCHIP_SOCK_NUM_] = {8, 8, 0, 0, 0, 0, 0, 0};

    if (wizchip_init(tx_size, rx_size) != 0)
    {
        ESP_LOGE(TAG, "wizchip_init failed");
        return ESP_FAIL;
    }

    wiz_NetInfo net_info = {
        WIZ_MAC,
        WIZ_IP,
        WIZ_SUBNET,
        WIZ_GATEWAY,
        WIZ_DNS,
        NETINFO_STATIC
    };

    if (ctlnetwork(CN_SET_NETINFO, &net_info) != 0)
    {
        ESP_LOGE(TAG, "W5500 network configuration failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t wiz_send(const uint8_t *data, size_t length)
{
    if (!data || length == 0)
        return ESP_ERR_INVALID_ARG;

    // send() writes data into the W5500's internal TX buffer.
    // The chip's hardwired TCP/IP stack handles segmentation and transmission.
    int32_t sent = wizsend(WIZ_SOCKET, (uint8_t *)data, (uint16_t)length);
    if (sent != (int32_t)length)
    {
        ESP_LOGE(TAG, "send() failed, returned %ld", (long)sent);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sent %zu bytes", length);
    return ESP_OK;
}

esp_err_t wiz_sendto(uint8_t *target_ip, uint8_t *data, uint8_t length)
{
    if (!data || length == 0)
        return ESP_ERR_INVALID_ARG;

    // send() writes data into the W5500's internal TX buffer.
    // The chip's hardwired TCP/IP stack handles segmentation and transmission.
    //uint8_t sn, uint8_t * buf, uint16_t len, uint8_t * addr, uint16_t port
    int32_t sent = wizsendto(WIZ_SOCKET, (uint8_t *)data, (uint16_t)length, data, REMOTE_PORT);
    if (sent != (int32_t)length)
    {
        ESP_LOGE(TAG, "send() failed, returned %ld", (long)sent);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sent %zu bytes", length);
    return ESP_OK;
}

esp_err_t wiz_receive(uint8_t *buf, size_t buf_size, size_t *bytes_read)
{
    if (!buf || !bytes_read)
        return ESP_ERR_INVALID_ARG;

    // Check how many bytes are waiting in the W5500's internal RX buffer
    uint16_t available = getSn_RX_RSR(WIZ_SOCKET);
    if (available == 0)
    {
        *bytes_read = 0;
        return ESP_ERR_NOT_FOUND; // No data yet
    }

    uint16_t to_read = (available < buf_size) ? available : (uint16_t)buf_size;

    int32_t result = wizrecv(WIZ_SOCKET, buf, to_read);
    if (result <= 0)
    {
        ESP_LOGE(TAG, "wizrecv() failed, returned %ld", (long)result);
        return ESP_FAIL;
    }

    *bytes_read = (size_t)result;
    ESP_LOGI(TAG, "Received %zu bytes", *bytes_read);
    return ESP_OK;
}

// In main: if(wiz_ensure_connected(ip, port) == ESP_OK){reconected}
// For reconnection also
esp_err_t wiz_ensure_connected(uint8_t *ip, uint16_t port)
{
    // Check W5500 socket status before attempting anything
    if (getSn_SR(WIZ_SOCKET) == SOCK_ESTABLISHED)
    {
        return ESP_OK; // Already connected, nothing to do
    }

    wiz_disconnect(); // Clean up any half-open socket first

    for (int attempt = 1; attempt <= 3; attempt++)
    {
        ESP_LOGI(TAG, "Connect attempt %d/3", attempt);
        if (wiz_connect(ip, port) == ESP_OK)
            return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    return ESP_FAIL; // Caller decides what to do after 3 failures
}
// ---------------------------------------------------------------------------
// wiz_ping — fire and forget: send and return IMMEDIATELY, no reply checked
// ---------------------------------------------------------------------------
esp_err_t wiz_ping(uint8_t *target_ip, const char *message)
{
    static uint16_t s_seq = 0;
    const uint16_t ID = 0xABCD;

    if (!target_ip || !message)
        return ESP_ERR_INVALID_ARG;
    ESP_LOGI(TAG, "Test ping to %d.%d.%d.%d",
             target_ip[0], target_ip[1], target_ip[2], target_ip[3]);

    if (wizsocket(WIZ_PING_SOCKET, Sn_MR_IPRAW, LOCAL_PORT, 0) != WIZ_PING_SOCKET)
    {
        ESP_LOGE(TAG, "ping: socket() failed");
        return ESP_FAIL;
    }

    icmp_packet_t request;
    build_icmp_request(&request, ID, ++s_seq, message);

    int32_t sent = wizsendto(
        WIZ_PING_SOCKET,
        (uint8_t *)&request,
        sizeof(request),
        target_ip,
        REMOTE_PORT);

    wizclose(WIZ_PING_SOCKET);
    //printf("%d\n", sent);

    if (sent != sizeof(request))
    {
        ESP_LOGE(TAG, "ping: wizsendto() failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ICMP echo sent to %d.%d.%d.%d",
             target_ip[0], target_ip[1], target_ip[2], target_ip[3]);
    return ESP_OK;
}

//esp_err_t wiz_ping(uint8_t *target_ip, const char *message)
//{
//    static uint16_t s_seq = 0;
//    const uint16_t ID = 0xABCD; //What is this ID?
//
//    if (wizsocket(WIZ_PING_SOCKET, Sn_MR_TCP, Sn_MR_IPRAW, 0) != WIZ_PING_SOCKET)
//    {
//        ESP_LOGE(TAG, "ping: socket() failed");
//        return ESP_FAIL;
//    }
//
//    //setSn_PROTO(WIZ_PING_SOCKET, 0x01);
//
//    icmp_packet_t request;
//    build_icmp_request(&request, ID, ++s_seq, message);
//
//    int32_t sent = wizsendto(WIZ_PING_SOCKET, (uint8_t *)&request,
//                          sizeof(request), target_ip, 5000);
//
//    // Close immediately — do not wait for any reply
//    wizclose(WIZ_PING_SOCKET);
//
//    if (sent != sizeof(request))
//    {
//        ESP_LOGE(TAG, "ping: wizsendto() failed");
//        return ESP_FAIL;
//    }
//
//    ESP_LOGI(TAG, "ping sent to %d.%d.%d.%d",
//             target_ip[0], target_ip[1], target_ip[2], target_ip[3]);
//    return ESP_OK;
//}

// Wait with this til we know if we or ground takes the initiative.
// Remember to update the .h file when decided.
// esp_err_t wiz_check_connection(uint8_t *buf)

// ---------------------------------------------------------------------------
// Connect/disconnet. This is for more than intitialize. It can also be used if
// fro example ground station crfaches and connection has to be reestablished.
// ---------------------------------------------------------------------------
esp_err_t wiz_connect(uint8_t *remote_ip, uint16_t remote_port)
{
    // Open socket 0 in TCP mode, using local port 5000 (arbitrary)
    if (wizsocket(WIZ_SOCKET, Sn_MR_TCP, 5000, 0) != WIZ_SOCKET)
    {
        ESP_LOGE(TAG, "socket() failed");
        return ESP_FAIL;
    }

    // Connect to remote host — W5500 handles the TCP handshake internally
    if (wizconnect(WIZ_SOCKET, remote_ip, remote_port) != SOCK_OK)
    {
        ESP_LOGE(TAG, "connect() failed");
        wizclose(WIZ_SOCKET);
        return ESP_FAIL;
    }

    TickType_t start = xTaskGetTickCount();
    while (getSn_SR(WIZ_SOCKET) != SOCK_ESTABLISHED)
    {
        if (getSn_IR(WIZ_SOCKET) & Sn_IR_TIMEOUT)
        {
            setSn_IR(WIZ_SOCKET, Sn_IR_TIMEOUT);
            wizclose(WIZ_SOCKET);
            ESP_LOGE(TAG, "TCP connect timed out");
            return ESP_ERR_TIMEOUT;
        }

        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(3000))
        {
            wizclose(WIZ_SOCKET);
            ESP_LOGE(TAG, "TCP connect did not establish in time");
            return ESP_ERR_TIMEOUT;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Connected to %d.%d.%d.%d:%d",
             remote_ip[0], remote_ip[1], remote_ip[2], remote_ip[3],
             remote_port);

    return ESP_OK;
}

void wiz_disconnect(void)
{
    wizdisconnect(WIZ_SOCKET);
    wizclose(WIZ_SOCKET);
    ESP_LOGI(TAG, "Disconnected");
}
