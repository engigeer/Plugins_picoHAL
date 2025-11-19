/*

  myspindle.c

  Part of grblHAL

  Copyright (c) 2025 Terje Io
  Copyright (c) 2025 Mitchell Grams

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "picohal.h"
#include "keypad.h"

#include "grbl/task.h"

static on_spindle_selected_ptr on_spindle_selected;
static on_keypress_preview_ptr on_keypress_preview;
static driver_reset_ptr driver_reset;

static spindle_id_t spindle_id;
static spindle_ptrs_t *spindle_hal = NULL;
static spindle_data_t spindle_data = {0};
static spindle_state_t spindle_state = {0};
static float saved_rpm;
static bool toggle_state = 1;

#ifndef PICOHAL_REG_SP_ENABLE
#define PICOHAL_REG_SP_ENABLE   0x0200
#endif

#ifndef PICOHAL_REG_SP_RPM
#define PICOHAL_REG_SP_RPM      0x0201
#endif

static void spindleSetRPM (float rpm, bool block)
{
    uint16_t rpm_value = (uint16_t)rpm; // convert float to integer

    if (!toggle_state)
        rpm = 0;

    modbus_message_t data = {
        .context = NULL,
        .crc_check = false,
        .adu[0] = PICOHAL_ADDRESS,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = (uint8_t)(PICOHAL_REG_SP_RPM >> 8),
        .adu[3] = (uint8_t)(PICOHAL_REG_SP_RPM & 0xFF),
        .adu[4] = (uint8_t)(rpm_value >> 8), // High byte
        .adu[5] = (uint8_t)(rpm_value & 0xFF), // Low byte
        .tx_length = 8,
        .rx_length = 8
    };

    picohal_send_message_now(&data, block);
}

static void spindleSetSpeed (spindle_ptrs_t *spindle, float rpm)
{
    UNUSED(spindle);

    spindleSetRPM(rpm, false);
    saved_rpm = rpm;
}

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    UNUSED(spindle);

    modbus_message_t data = {
        .context = NULL,
        .crc_check = false,
        .adu[0] = PICOHAL_ADDRESS,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = (uint8_t)(PICOHAL_REG_SP_ENABLE >> 8),
        .adu[3] = (uint8_t)(PICOHAL_REG_SP_ENABLE & 0xFF),
        .adu[4] = 0x00,
        .adu[5] = (!state.on) ? 0x00 : (state.ccw ? 0x03 : 0x01),
        .tx_length = 8,
        .rx_length = 8
    };

    spindle_state.on = state.on;
    spindle_state.ccw = state.ccw;
    toggle_state = 1;

    if(picohal_send_message_now(&data, false))
        spindleSetRPM(rpm, false);
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    UNUSED(spindle);

    return spindle_state;
}

// static spindle_data_t *spindleGetData (spindle_data_request_t request)
// {
//     return &spindle_data;
// }

static void onSpindleSelected (spindle_ptrs_t *spindle)
{
    if(spindle->id == spindle_id) {

        spindle_hal = spindle;
        spindle_data.rpm_programmed = -1.0f;

        //modbus_set_silence(NULL);

    } else
        spindle_hal = NULL;

    if(on_spindle_selected)
        on_spindle_selected(spindle);
}

static bool keypress_preview (char c, uint_fast16_t state)
{
    // OVERRIDES DEFAULT SPINDLE STOP BEHAVIOUR (ONLY FOR COMMANDS FROM KEYPAD)
    if(c == CMD_OVERRIDE_SPINDLE_STOP && spindle_state.on){
        
        toggle_state = !toggle_state;

        if (toggle_state)
            spindleSetRPM(saved_rpm, false);
        else
            spindleSetRPM(0, false);
        return true;
    }
    else
        return on_keypress_preview && on_keypress_preview(c, state);
}

static bool spindleConfig (spindle_ptrs_t *spindle)
{
    return modbus_isup().rtu;
}

static const spindle_ptrs_t spindle = {
    .type = SpindleType_VFD, //TODO ADD CUSTOM SPINDLE TYPE
    .ref_id = SPINDLE_MY_SPINDLE,
    .cap = {
        .variable = On,
        .at_speed = Off,
        .direction = Off,
        .cmd_controlled = On,
        .laser = On
    },
    .config = spindleConfig,
    .set_state = spindleSetState,
    .get_state = spindleGetState,
    .update_rpm = spindleSetSpeed
};

static void OnReset (void)
{
    // Ensure spindle is off on RESET (TODO: also set power to zero?, also is this correct? how does it interact with iosender . . .)
    spindle_all_off();

    driver_reset();
}

void picohal_spindle_init (void)
{
    // INIT PICOHAL SPINDLE IF CONFIGURED
    #if SPINDLE_ENABLE & (1<<SPINDLE_MY_SPINDLE)

        if((spindle_id = spindle_register(&spindle, "PicoHAL")) != -1) {
            // spindleSetState(NULL, spindle_state, 0.0f);

            on_spindle_selected = grbl.on_spindle_selected;
            grbl.on_spindle_selected = onSpindleSelected;

            #if KEYPAD_ENABLE
                on_keypress_preview = keypad.on_keypress_preview;
                keypad.on_keypress_preview = keypress_preview;
            #endif
            
            driver_reset = hal.driver_reset;
            hal.driver_reset = OnReset;

        } else {
            task_add_immediate(report_warning, "PicoHAL spindle failed to initialize!");
        }
    #endif  

}