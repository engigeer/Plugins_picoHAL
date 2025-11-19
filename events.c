/*
  events.c

  Part of grblHAL

  Copyright (c) 2023 Expatria Technologies Inc.
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

#include "picohal.h"

#ifndef PICOHAL_REG_STATE
#define PICOHAL_REG_STATE       0x0001
#endif

#ifndef PICOHAL_REG_ALARM
#define PICOHAL_REG_ALARM       0x0002
#endif

#ifndef PICOHAL_REG_COOLANT
#define PICOHAL_REG_COOLANT     0x0003
#endif

#ifndef PICOHAL_REG_EVENT
#define PICOHAL_REG_EVENT       0x0004
#endif

static coolant_set_state_ptr on_coolant_changed;
static on_state_change_ptr on_state_change;

static on_homing_completed_ptr on_homing_completed;
static on_probe_completed_ptr on_probe_completed;
static on_probe_start_ptr on_probe_start;
static on_probe_toolsetter_ptr on_probe_toolsetter;
static on_program_completed_ptr on_program_completed;

typedef enum {
    TOOLCHANGE_ACK = 0,
    PROBE_START = 1,
    PROBE_COMPLETED = 2,
    PROBE_FIXTURE = 3,
    PROGRAM_COMPLETED = 30,
    HOMING_COMPLETED = 31,
    INVALID_EVENT = 255,
} picohal_events;

static sys_state_t current_machine_state;
static uint8_t current_coolant_state;
static uint8_t last_alarm;

static void onStateChanged (sys_state_t state)
{    
    system_state_t machine_state = ffs(state);
    alarm_code_t alarm_code;

    if(state & (STATE_ESTOP|STATE_ALARM)) {
        char *alarm;

        machine_state = SystemState_Alarm;
        alarm_code = state_get_substate();
    }
    else {
        last_alarm = Alarm_None; // reset alarm code
    }

    if (picohal_is_online && (machine_state != current_machine_state)){

            current_machine_state = machine_state;

            modbus_message_t data = {
            .context = NULL,
            .crc_check = false,
            .adu[0] = PICOHAL_ADDRESS,
            .adu[1] = ModBus_WriteRegister,
            .adu[2] = (uint8_t)(PICOHAL_REG_STATE >> 8),
            .adu[3] = (uint8_t)(PICOHAL_REG_STATE & 0xFF),
            .adu[4] = (uint8_t)(machine_state >> 8),
            .adu[5] = (uint8_t)(machine_state & 0xFF),
            .tx_length = 8,
            .rx_length = 8
        };

        picohal_send_message_now(&data, false);
    }

    if (picohal_is_online && alarm_code && (alarm_code != last_alarm)) {

            last_alarm = alarm_code;

            modbus_message_t data = {
            .context = NULL,
            .crc_check = false,
            .adu[0] = PICOHAL_ADDRESS,
            .adu[1] = ModBus_WriteRegister,
            .adu[2] = (uint8_t)(PICOHAL_REG_ALARM >> 8),
            .adu[3] = (uint8_t)(PICOHAL_REG_ALARM & 0xFF),
            .adu[4] = (uint8_t)(alarm_code >> 8),
            .adu[5] = (uint8_t)(alarm_code & 0xFF),
            .tx_length = 8,
            .rx_length = 8
        };

        picohal_send_message_now(&data, false);
    }
    
    if(on_state_change)
        on_state_change(state);
}

static void onCoolantChanged (coolant_state_t state){

    uint8_t coolant_state = state.value;

    if (picohal_is_online && (coolant_state != current_coolant_state)) {

            current_coolant_state = coolant_state;

            modbus_message_t data = {
            .context = NULL,
            .crc_check = false,
            .adu[0] = PICOHAL_ADDRESS,
            .adu[1] = ModBus_WriteRegister,
            .adu[2] = (uint8_t)(PICOHAL_REG_COOLANT >> 8),
            .adu[3] = (uint8_t)(PICOHAL_REG_COOLANT & 0xFF),
            .adu[4] = (uint8_t)(coolant_state >> 8),
            .adu[5] = (uint8_t)(coolant_state & 0xFF),
            .tx_length = 8,
            .rx_length = 8
        };

        picohal_send_message_now(&data, false);

    }

    if (on_coolant_changed)         // Call previous function in the chain.
        on_coolant_changed(state);    
}

static void picohal_create_event (picohal_events event){

    if (picohal_is_online) {

        modbus_message_t data = {
        .context = NULL,
        .crc_check = false,
        .adu[0] = PICOHAL_ADDRESS,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = (uint8_t)(PICOHAL_REG_EVENT >> 8),
        .adu[3] = (uint8_t)(PICOHAL_REG_EVENT & 0xFF),
        .adu[4] = (uint8_t)(event >> 8),
        .adu[5] = (uint8_t)(event & 0xFF),
        .tx_length = 8,
        .rx_length = 8
        };
    
        picohal_send_message_now(&data, false);

    }
}

static void onHomingCompleted (axes_signals_t cycle, bool success)
{
    if (success)
        // cycle contains the axis flags of the executed homing cycle
        //success will be true when all the configured cycles are completed.
        picohal_create_event(HOMING_COMPLETED);
    
    if(on_homing_completed)
        on_homing_completed(cycle, success);
}

static bool onProbeStart (axes_signals_t axes, float *target, plan_line_data_t *pl_data)
{
    //write the program flow value to the event register.
    picohal_create_event(PROBE_START);
    
    if(on_probe_start)
        on_probe_start(axes, target, pl_data);

    return false;
}

static void onProbeCompleted ()
{
    //write the program flow value to the event register.
    picohal_create_event(PROBE_COMPLETED);
    
    if(on_probe_completed)
        on_probe_completed();
}

static bool onProbeToolsetter (tool_data_t *tool, coord_data_t *position, bool at_g59_3, bool on)
{
    //write the program flow value to the event register.
    picohal_create_event(PROBE_FIXTURE);
    
    if(on_probe_toolsetter)
        on_probe_toolsetter(tool, position, at_g59_3, on);

    return false;
}

static void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{
    //write the program flow value to the event register.
    picohal_create_event(PROGRAM_COMPLETED);
    
    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

void picohal_events_init (void)
{
        on_state_change = grbl.on_state_change;
        grbl.on_state_change = onStateChanged;

        on_coolant_changed = hal.coolant.set_state;         //subscribe to coolant events
        hal.coolant.set_state = onCoolantChanged;

        on_homing_completed = grbl.on_homing_completed;     //subscribe to homing completed
        grbl.on_homing_completed = onHomingCompleted; 

        on_probe_start = grbl.on_probe_start;               //subscribe to probe start
        grbl.on_probe_start = onProbeStart;

        on_probe_completed = grbl.on_probe_completed;       //subscribe to probe completed
        grbl.on_probe_completed = onProbeCompleted; 

        on_probe_toolsetter = grbl.on_probe_toolsetter;     //subscribe to probe start
        grbl.on_probe_toolsetter = onProbeToolsetter;

        on_program_completed = grbl.on_program_completed;   // Subscribe to on program completed events
        grbl.on_program_completed = onProgramCompleted;     // Checkered Flag for successful end of program lives here
}