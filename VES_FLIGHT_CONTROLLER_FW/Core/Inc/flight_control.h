/*
 * FlightControl.h
 *
 *  Created on: Apr 2, 2024
 *      Author: tommaso
 * 		Contributor: Vincenzo Catalano
 */

#ifndef FLIGHTCONTROL_H
#define FLIGHTCONTROL_H

#ifndef __MAIN_H
#include "main.h"
#endif

#ifndef FLIGHT_PARAMETERS_CONFIG_H
#include "FLIGHT_PARAMETERS_CONFIG.h"
#endif

extern uint16_t memoryBit;

typedef struct{
	float_t accX;
	float_t accY;
	float_t accZ;
} linear_acceleration_t;

typedef enum {
	INVALID = 0,
	CALIBRATING = 1,
	READY,
	BURNING,
	COASTING,
	DROGUE,
	MAIN,
	TOUCHDOWN
} flight_phase_t;

typedef enum {
	EV_CALIBRATE = 0,
	EV_READY,
	EV_LIFTOFF,
	EV_MAX_V,
	EV_APOGEE,			/* open the drogue parachute */
	EV_MAIN_DEPLOYMENT,	/* open the main parachute */
	EV_TOUCHDOWN		/* deinit servo, and other stuffs, stop flash storing */
} events_trigger;


void check_system_calibrating_phase(flight_phase_t *, linear_acceleration_t acc_data, linear_acceleration_t , linear_acceleration_t gyro_data, linear_acceleration_t);
void check_ready_to_takeoff_phase(flight_phase_t *, linear_acceleration_t acc_data);
void check_burning_phase(flight_phase_t *, linear_acceleration_t );
void check_coasting_phase(flight_phase_t *, linear_acceleration_t);
void check_drogue_deploy_phase(flight_phase_t *, linear_acceleration_t);
void check_main_deploy_phase(flight_phase_t *phase, linear_acceleration_t acc_data);
void check_touchdown_phase(flight_phase_t *, linear_acceleration_t);

void check_flight_phase(flight_phase_t *, linear_acceleration_t acc_data, linear_acceleration_t gyro_data, linear_acceleration_t, linear_acceleration_t);

void change_state_to(flight_phase_t *old_phase, flight_phase_t new_state, events_trigger event_to_trigger);
void clear_memory_bit(uint16_t *);


#endif /* INC__FLIGHTCONTROL_H_ */

