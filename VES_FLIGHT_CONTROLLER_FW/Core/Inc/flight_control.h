/*
 * FlightControl.h
 *
 *  Created on: Apr 2, 2024
 *      Author: tommaso & Francesco Abate
 */

#ifndef FLIGHTCONTROL_H
#define FLIGHTCONTROL_H

#ifndef __MAIN_H
#include "main.h"
#endif

#include "utilities.h"
#include <stdio.h>
#include <stdbool.h>
#include "W25Q128.h"

#ifndef FLIGHT_PARAMETERS_CONFIG_H
#include "FLIGHT_PARAMETERS_CONFIG.h"
#endif




typedef enum {
	INVALID,
	CALIBRATING,
	READY,
	BURNING,
	COASTING,
	DROGUE,
	MAIN,
	TOUCHDOWN
} flight_phase_t;

typedef struct{
	float_t velX;
	float_t velY;
	float_t velZ;
} linear_velocity_t;

typedef struct{
	float_t accX;
	float_t accY;
	float_t accZ;
} linear_acceleration_t;

typedef struct{
  float_t height;        // m
  float_t velocity;      // m/s
  float_t acceleration;  // m/s^2
} estimation_output_t ;



typedef struct  {
	  flight_phase_t flight_state;
	  linear_acceleration_t old_acc_data;
	  linear_acceleration_t old_gyro_data;
	  uint32_t clock_memory;
	  uint32_t memory[3];
	  uint32_t thrust_trigger_time;
	  bool state_changed;
	}flight_fsm_t;


typedef enum {
	EV_INVALID = 0,
	EV_CALIBRATING,
	EV_READY,
	EV_BURNING,
	EV_COASTING,
	EV_DROGUE,
	EV_MAIN,
	EV_TOUCHDOWN
} events_trigger;


osStatus_t trigger_event(events_trigger ev, bool event_unique);


void check_Calibrating_phase(flight_fsm_t *fsm_state,estimation_output_t MotionData,linear_acceleration_t acc_data, linear_acceleration_t gyro_data);
void check_Ready_phase(flight_fsm_t *fsm_state,estimation_output_t MotionData, linear_acceleration_t acc_data);
void check_Burning_phase(flight_fsm_t *fsm_state, estimation_output_t MotionData);
void check_Coasting_phase(flight_fsm_t *fsm_state,estimation_output_t MotionData);
void check_Drogue_phase(flight_fsm_t *fsm_state,estimation_output_t MotionData);
void check_Main_phase(flight_fsm_t *fsm_state,estimation_output_t MotionData);
void check_Touchdown_phase(flight_fsm_t *fsm_state,estimation_output_t MotionData, linear_acceleration_t acc_data);

void check_flight_phase(flight_fsm_t *fsm_state,estimation_output_t MotionData,linear_acceleration_t acc_data, linear_acceleration_t gyro_data);

void change_state_to(flight_fsm_t *phase, flight_phase_t new_state, events_trigger event_to_trigger);



#endif /* INC__FLIGHTCONTROL_H_ */

