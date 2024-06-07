/*
 * FlightControl.h
 *
 *  Created on: Apr 2, 2024
 *      Author: tommaso
 * 		Contributor: Vincenzo Catalano
 */
#include <math.h>
#include <stdint.h>
#include <main.h>

#ifndef FLIGHTCONTROL_H
#define FLIGHTCONTROL_H

extern uint16_t memoryBit;
extern const float ACC_SENSOR_ERROR;
extern const float ALLOWED_ACC_ERROR;
extern const float ALLOWED_GYRO_ERROR;
extern const float MAIN_SAFETY_COUNTER;
extern const float COASTING_SAFETY_COUNTER;
extern const uint16_t LIFTOFF_SAFETY_COUNTER;
extern const uint16_t TOUCHDOWN_SAFETY_COUNTER;
extern const float VELOCITY_BOUND_TOUCHDOWN;
extern const uint16_t TOUCHDOWN_SAFETY_COUNTER;
extern const uint16_t APOGEE_SAFETY_COUNTER;
extern const uint16_t TRESHOLD;
extern const float LIFT_OFF_ACC_TRESHOLD;

typedef struct{
	float_t acc1;
	float_t acc2;
	float_t acc3;
} Acc;
typedef struct{
	float_t vel;
	float_t vel2;
	float_t vel3;
} Vel;

//Acc old_acc_data;
//Acc new_acc_data;
//Acc old_gyro_data;
//Acc new_gyro_data;


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


void check_flight_phase(Phase *, Acc acc_data, Acc gyro_data,Acc,Acc, Vel);
 void check_system_calibrating_phase(Phase *, Acc acc_data, Acc , Acc gyro_data,Acc);
 void check_ready_takeoff_phase(Phase *, Acc acc_data);
 void check_burning_phase(Phase *,Acc );
 void check_coasting_phase(Phase *,Acc);
void check_drogue_deploy_phase(Phase *,Acc);
 void check_main_deploy_phase(Phase *,Acc,Vel);
void check_touchdown_phase(Phase *,Acc);
void change_state_to(Phase *old_phase,Phase new_state, Events_trigger event_to_trigger);
void clear_memory_bit(uint16_t *);


#endif /* INC__FLIGHTCONTROL_H_ */

