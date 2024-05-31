/*
 * FlightControl.h
 *
 *  Created on: Apr 2, 2024
 *      Author: tommaso
 * 		Contributor: Vincenzo Catalano
 */
#include <math.h>
#include <stdint.h>

#ifndef FLIGHTCONTROL_H
#define FLIGHTCONTROL_H
typedef struct {
	float x, y, z;
}Acc;

 Acc old_acc_data;
 Acc new_acc_data;
 Acc old_gyro_data;
 Acc new_gyro_data;
uint16_t memoryBit=0;
const float ACC_SENSOR_ERROR=0;
const float ALLOWED_ACC_ERROR;
const float ALLOWED_GYRO_ERROR;
const uint16_t TRESHOLD=10;

typedef enum {
	INVALID = 0,
	CALIBRATING = 1,
	READY,
	BURNING,
	COASTING,
	DROGUE,
	MAIN,
	TOUCHDOWN
}Phase ;

typedef enum {
  EV_CALIBRATE = 0,
  EV_READY,
  EV_LIFTOFF,
  EV_MAX_V,
  EV_APOGEE,
  EV_MAIN_DEPLOYMENT,
  EV_TOUCHDOWN,
  EV_CUSTOM_1,
  EV_CUSTOM_2
} Events_trigger;


void check_flight_phase(Phase *, Acc acc_data, Acc gyro_data, estimation_output_t state_data);
static void check_system_calibrating_phase(Phase *, Acc acc_data, Acc gyro_data);
static void check_ready_takeoff_phase(Phase *, Acc acc_data);
static void check_burning_phase(Phase *,Acc, estimation_output_t state_data);
static void check_coasting_phase(Phase *,Acc, estimation_output_t state_data);
static void check_drogue_deploy_phase(Phase *,Acc, estimation_output_t state_data);
static void check_main_deploy_phase(Phase *,Acc, estimation_output_t state_data);
static void check_touchdown_phase(Phase *,Acc, estimation_output_t state_data);
static void change_state_to(Phase *old_phase,Phase new_state, Events_trigger event_to_trigger);
static void clear_memory_bit(uint16_t *);


#endif /* INC__FLIGHTCONTROL_H_ */

