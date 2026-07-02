#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "socket.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief  Open a TCP connection to a remote host.
     *
     * @param[in] remote_ip    Remote IP as 4-byte array, e.g. {192,168,1,50}
     * @param[in] remote_port  Remote port number.
     *
     * @return ESP_OK on success, or an esp_err_t error code.
     */
    esp_err_t wiz_connect(uint8_t *remote_ip, uint16_t remote_port);

    /**
     * @brief  Send data over the open TCP connection.
     *         Data is written into the W5500's internal TX buffer and
     *         transmitted by the chip's hardwired TCP/IP stack.
     *
     * @param[in] data    Pointer to the buffer to send.
     * @param[in] length  Number of bytes to send.
     *
     * @return ESP_OK on success, or an esp_err_t error code.
     */
    esp_err_t wiz_send(const uint8_t *data, size_t length);

    /**
     * @brief  Receive data from the W5500's internal RX buffer.
     *
     * @param[out] buf         Buffer to write received data into.
     * @param[in]  buf_size    Size of buf in bytes.
     * @param[out] bytes_read  Actual number of bytes received.
     *
     * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no data available,
     *         or another esp_err_t on error.
     */
    esp_err_t wiz_receive(uint8_t *buf, size_t buf_size, size_t *bytes_read);

    /**
     * @brief  Check if the TCP connection is established, and if not, attempt to reconnect.
     *
     * @param[in] remote_ip    Remote IP as 4-byte array, e.g. {192,168,1,50}
     * @param[in] remote_port  Remote port number.
     *
     * @return ESP_OK if already connected or successfully reconnected,
     *         or an esp_err_t on error.
     */
    esp_err_t wiz_ensure_connected(uint8_t *ip, uint16_t port);

    /**
     * @brief  Send an ICMP Echo Request and return IMMEDIATELY.
     *         Fire-and-forget — no reply is waited for or checked.
     *         Useful for watchdog-style "I am alive" signals where
     *         the receiver logs arrival but the sender does not block.
     *
     * @param[in] target_ip  IP to ping as 4-byte array, e.g. {192,168,1,1}
     * @param[in] message    Message to send with ping. Max 32 bytes.
     *
     * @return ESP_OK if the packet was handed to the W5500 TX buffer,
     *         or an esp_err_t on error.
     */
    esp_err_t wiz_ping(uint8_t *target_ip, const char *message);

    /**
     * @brief  Close the TCP connection and release the socket.
     * 
     * @return wiz
     */
    void wiz_disconnect(void);

#ifdef __cplusplus
}
#endif
