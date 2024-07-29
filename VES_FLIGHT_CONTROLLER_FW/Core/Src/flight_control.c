/*
 * FlightControl.c
 *
 *  Created on: Apr 2, 2024
 *      Author: tommaso
 * 		Contributor: Vincenzo Catalano
 */

#ifndef FLIGHTCONTROL_H
#include "flight_control.h"
#endif

uint16_t memoryBit = 0;

void check_flight_phase(flight_phase_t *phase, linear_acceleration_t acc_data, linear_acceleration_t old_acc_data, linear_acceleration_t gyro_data, linear_acceleration_t old_gyro_data) {

	 switch (*phase) {

	    case CALIBRATING:
			  check_system_calibrating_phase(phase, acc_data, old_acc_data, gyro_data, old_gyro_data);
			  break;

	    case READY:
			  check_ready_to_takeoff_phase(phase, acc_data);
			  break;

	    case BURNING:
			  check_burning_phase(phase, acc_data);
			  break;

	    case COASTING:
			  check_coasting_phase(phase, acc_data);
			  break;

	    case DROGUE:
			  check_drogue_deploy_phase(phase, acc_data);
			  break;

	    case MAIN:
			  check_main_deploy_phase(phase, acc_data);
			  break;

	    case TOUCHDOWN:
	    		check_touchdown_phase(phase, acc_data);
	    	break;

	    case INVALID:
	    	break;

	  }
}

void check_system_calibrating_phase(flight_phase_t *phase, linear_acceleration_t acc_data, linear_acceleration_t old_acc_data, linear_acceleration_t gyro_data, linear_acceleration_t old_gyro_data) {
	// if I'm in a higher state I cannot come back
	if(*phase > CALIBRATING) return;

	if ((fabsf(old_acc_data.accX - acc_data.accX) < ACCELEROMETER_MEASURE_TOLERANCE_MG) &&
	  (fabsf(old_acc_data.accY - acc_data.accY) < ACCELEROMETER_MEASURE_TOLERANCE_MG) &&
	  (fabsf(old_acc_data.accZ - acc_data.accZ) < ACCELEROMETER_MEASURE_TOLERANCE_MG) &&
	  (fabsf(old_gyro_data.accX - gyro_data.accX) < GYRO_MEASURE_TOLERANCE_MDPS) &&
	  (fabsf(old_gyro_data.accY - gyro_data.accY) < GYRO_MEASURE_TOLERANCE_MDPS) &&
	  (fabsf(old_gyro_data.accZ - gyro_data.accZ) < GYRO_MEASURE_TOLERANCE_MDPS)) {

		memoryBit++;
	} else {

		memoryBit = 0;
	}

    old_acc_data = acc_data;
  	old_gyro_data = gyro_data;

	if(memoryBit > THRESHOLD) {
		/*
		 * parameter 1 : phase variable
		 * parameter 2 : new phase
		 * parameter 3 : events to trigger
		 */
		change_state_to(phase, READY, EV_READY);
	}

}

void check_ready_to_takeoff_phase(flight_phase_t *phase, linear_acceleration_t acc_data) {
	// if I'm in a higher state I cannot come back

	if(*phase > READY) return;

	const float accel_x = acc_data.accX * acc_data.accX;
	const float accel_y = acc_data.accY * acc_data.accY;
	const float accel_z = acc_data.accZ * acc_data.accZ;
	const float acceleration = accel_x + accel_y + accel_z;

	// do we use? or can we use a flag instead?
	//	state->memory[1] > LIFTOFF_SAFETY_COUNTER
	if (acceleration >  LIFT_OFF_ACC_THRESHOLD) {
		memoryBit++;
	  } else {
		memoryBit = 0;
	  }

	if (memoryBit > LIFTOFF_SAFETY_COUNTER) {
		/*
		 * parameter 1 : phase variable
		 * parameter 2 : new phase
		 * parameter 3 : events to trigger
		 */
		change_state_to(phase, BURNING, EV_LIFTOFF);
	}

}

void check_burning_phase(flight_phase_t *phase, linear_acceleration_t acc_data) {
	// if I'm in a higher state I cannot come back
	if(*phase > BURNING) return;

	memoryBit = 0;
	 /* When acceleration is below 0, liftoff concludes */

	//XXX
	//FIXME be sure that the acceleration checked is the right one
	// remember that when on the rocket the gravity acceleration works on the x axis
	if (acc_data.accZ > LIFT_OFF_ACC_THRESHOLD) {
		// LIFT_OFF_ACC_THRESHOLD is not the correct value to check against
		memoryBit++;
	} else {
		memoryBit = 0;
	}

	if (memoryBit > COASTING_SAFETY_COUNTER) {
		change_state_to(phase, COASTING, EV_MAX_V);
	}
}

void check_coasting_phase(flight_phase_t *phase, linear_acceleration_t acc_data) {
	// if I'm in a higher state I cannot come back
	if(*phase > COASTING) return;

	memoryBit = 0;
	/* When velocity is below 0, coasting concludes */
	// if we need to check acceleration be sure that accZ is the correct one to check
	//XXX check the altitude for a specific amount of time
	if (acc_data.accZ < 0) {
		memoryBit++;
	}

	if (memoryBit > APOGEE_SAFETY_COUNTER) {
		/* If the duration between BURNING and apogee is smaller than defined, go to touchdown */
	/*	if ((osKernelGetTickCount() - memoryBitthrust_trigger_time) < MIN_TICK_COUNTS_BETWEEN_BURNING_APOGEE) {
			change_state_to(TOUCHDOWN, EV_TOUCHDOWN, fsm_state);
		} else {
			change_state_to(DROGUE, EV_APOGEE, fsm_state);
		}*/
	}

}

void check_drogue_deploy_phase(flight_phase_t *phase, linear_acceleration_t acc_data) {
	// if I'm in a higher state I cannot come back
	if(*phase > DROGUE) return;

	memoryBit = 0;

	if (1) {/* check the current altitude in order to decide whether to deploy the drogue or not */
		/* Achieved Height to deploy Main */
		memoryBit++;
	} else {
		/* Did Not Achieve */
		memoryBit = 0;
	}

	if (memoryBit > MAIN_SAFETY_COUNTER) {
		change_state_to(phase, MAIN, EV_MAIN_DEPLOYMENT);
	}

}

void check_main_deploy_phase(flight_phase_t *phase, linear_acceleration_t acc_data) {
	// if I'm in a higher state I cannot come back
	if(*phase > MAIN) return;

	memoryBit = 0;

	/* If the velocity is very small we have touchdown */
	// check the altitude for a specific amount of time
//	if () {
//		/* Touchdown achieved */
//		memoryBit++;
//	} else {
//		/* Touchdown not achieved */
//		memoryBit = 0;
//	}

	if (memoryBit > TOUCHDOWN_SAFETY_COUNTER) {
		change_state_to(phase, TOUCHDOWN, EV_TOUCHDOWN);
	}

}

void check_touch_down_phase(flight_phase_t *phase, linear_acceleration_t acc_data) {
	// if I'm in a higher state I cannot come back
	if(*phase > TOUCHDOWN) return;

	memoryBit = 0;

	//XXX
	//FIXME maybe put a difference check like in check_system_calibrating_phase
	// using both the last and last - 1 measurements to check the difference

	if ((fabsf(acc_data.accX ) < ACCELEROMETER_MEASURE_TOLERANCE_MG) &&
		(fabsf(acc_data.accY) < ACCELEROMETER_MEASURE_TOLERANCE_MG) &&
		(fabsf(acc_data.accZ) < ACCELEROMETER_MEASURE_TOLERANCE_MG)) {
		/* Touchdown achieved */
		memoryBit++;
	} else {
		/* Touchdown not achieved */
		memoryBit = 0;
	}

	if (memoryBit > TOUCHDOWN_SAFETY_COUNTER) {
		change_state_to(phase, TOUCHDOWN, EV_TOUCHDOWN);
	}
}

void clear_memory_bit(uint16_t *value) {
	*value = 0;
}

 void change_state_to(flight_phase_t *old_phase, flight_phase_t new_phase, events_trigger event_to_trigger) {
/*	if (flight_state == BURNING) {
		thrust_trigger_time = osKernelGetTickCount();
	}

	trigger_event(event_to_trigger);
	osEventFlagsClear(fsm_flag_id, 0xFF);
	osEventFlagsSet(fsm_flag_id, new_state);
	//go to
	*old_phase=new_phase;
	clear_memory_bit(&memoryBit);*/
	int c=1;
}
