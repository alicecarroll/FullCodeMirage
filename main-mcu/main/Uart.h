#pragma once
#include <stddef.h>
#include <stdint.h>

void K96_on();

void K96_off();

void K96_send(const char *data);

int K96_read(char *buffer,
             size_t max_length,
             uint32_t timeout_ms);

int K96_request(const char *command,
                char *response,
                size_t response_size,
                uint32_t timeout_ms);

void read_k96();