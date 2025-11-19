/*
  ioports.h - connect to picoHAL (RP2040) ioexpander via modbus RTU

  Part of grblHAL

  Copyright (c) 2023 Expatria Technologies Inc.
  Copyright (c) 2025 Terje Io
  Copyright (c) 2025 Mitchell Grams

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "driver.h"

#include "grbl/modbus.h"

#ifndef PICOHAL_ADDRESS
#define PICOHAL_ADDRESS               10
#endif

#define PICOHAL_RETRIES               5
#define PICOHAL_RETRY_DELAY           250
#define PICOHAL_KEEPALIVE_INTERVAL    1000

#ifndef PICOHAL_REG_KEEPALIVE
#define PICOHAL_REG_KEEPALIVE   0x0100
#endif

#ifndef PICOHAL_REG_DOUT
#define PICOHAL_REG_DOUT        0x0110
#endif

#ifndef PICOHAL_REG_AOUT
#define PICOHAL_REG_AOUT        0x0120
#endif

typedef enum {
    PICOHAL_MSG_KEEPALIVE = 0
} picohal_response_t;

static modbus_message_t keepalive_msg = {
    .context = PICOHAL_MSG_KEEPALIVE,
    .crc_check = false,
    .adu[0] = PICOHAL_ADDRESS,
    .adu[1] = ModBus_WriteRegister,
    .adu[2] = (uint8_t)(PICOHAL_REG_KEEPALIVE >> 8),
    .adu[3] = (uint8_t)(PICOHAL_REG_KEEPALIVE & 0xFF),
    .adu[4] = 0,
    .adu[5] = 0x01,
    .tx_length = 8,
    .rx_length = 8
};

static modbus_message_t reset_msg = {
    .context = NULL,
    .crc_check = false,
    .adu[0] = PICOHAL_ADDRESS,
    .adu[1] = ModBus_WriteRegister,
    .adu[2] = (uint8_t)(PICOHAL_REG_DOUT >> 8),
    .adu[3] = (uint8_t)(PICOHAL_REG_DOUT & 0xFF),
    .adu[4] = 0,
    .adu[5] = 0,
    .tx_length = 8,
    .rx_length = 8
};

static void picohal_rx_packet (modbus_message_t *msg);
static void picohal_rx_exception (uint8_t code, void *context);
static bool picohal_is_online;

static const modbus_callbacks_t callbacks = {
    .retries = PICOHAL_RETRIES,
    .retry_delay = PICOHAL_RETRY_DELAY,    
    .on_rx_packet = picohal_rx_packet,
    .on_rx_exception = picohal_rx_exception
};

typedef struct {
    uint16_t index;
    modbus_message_t picohal_packet;
} QueueItem;

typedef struct {
    uint16_t addr;
    xbar_t aux;
} picohal_aux_t;

bool picohal_send_message_now(modbus_message_t *data, bool block);

void picohal_spindle_init(void);
void picohal_events_init(void);