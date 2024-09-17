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
	INVALID = 0,
	CALIBRATING = 1,
	READY,
	JELQING,
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
	EV_CALIBRATE = 0,
	EV_READY,
	EV_EDGING,
	EV_LIFTOFF,
	EV_MAX_V,
	EV_APOGEE,			/* open the drogue parachute */
	EV_MAIN_DEPLOYMENT,	/* open the main parachute */
	EV_TOUCHDOWN		/* deinit servo, and other stuffs, stop flash storing */
} events_trigger;


osStatus_t trigger_event(events_trigger ev, bool event_unique);


void check_system_calibrating_phase(flight_fsm_t *fsm_state,estimation_output_t MotionData,linear_acceleration_t acc_data, linear_acceleration_t gyro_data);
void check_ready_to_takeoff_phase(flight_fsm_t *fsm_state,estimation_output_t MotionData, linear_acceleration_t acc_data);
void check_jelqing_phase(flight_fsm_t *fsm_state, estimation_output_t MotionData);
void check_burning_phase(flight_fsm_t *fsm_state,estimation_output_t MotionData);
void check_coasting_phase(flight_fsm_t *fsm_state,estimation_output_t MotionData);
void check_drogue_deploy_phase(flight_fsm_t *fsm_state,estimation_output_t MotionData);
void check_main_deploy_phase(flight_fsm_t *fsm_state,estimation_output_t MotionData);
void check_touch_down_phase(flight_fsm_t *fsm_state,estimation_output_t MotionData, linear_acceleration_t acc_data);

void check_flight_phase(flight_fsm_t *fsm_state,estimation_output_t MotionData,linear_acceleration_t acc_data, linear_acceleration_t gyro_data);

void change_state_to(flight_fsm_t *phase, flight_phase_t new_state, events_trigger event_to_trigger);



#endif /* INC__FLIGHTCONTROL_H_ */

