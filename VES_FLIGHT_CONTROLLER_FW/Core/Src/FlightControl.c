/*
 * FlightControl.c
 *
 *  Created on: Apr 2, 2024
 *      Author: tommaso
 * 		Contributor: Vincenzo Catalano
 */

#ifndef FLIGHTCONTROL_H
#include "FlightControl.h"

void check_flight_phase(Phase *phase, Acc acc_data, Acc gyro_data, estimation_output_t state_data){
	 switch (*phase) {
	    case CALIBRATING:
			  check_system_calibrating_phase( phase, acc_data, gyro_data);
			  break;
	    case READY:
			  check_ready_takeoff_phase( phase, acc_data);
			  break;
	    case BURNING:
			  check_burning_phase( phase, state_data);
			  break;
	    case COASTING:
			  check_coasting_phase( phase, state_data);
			  break;
	    case DROGUE:
			  check_drogue_deploy_phase( phase, state_data);
			  break;
	    case MAIN:
			  check_main_deploy_phase( phase, state_data);
			  break;
	    case TOUCHDOWN:
	    		check_touchdown_phase( phase, acc_data, gyro_data);
	    	break;
	    case INVALID:
	    	break;

	  }
}

static void check_system_calibrating_phase(Phase *phase , Acc acc_data, Acc gyro_data){
	// if I'm in a higher state I cannot come back
	if(*phase>CALIBRATING) return;

	if ((fabsf(old_acc_data.x - acc_data.x) < ALLOWED_ACC_ERROR) &&
	  (fabsf(old_acc_data.y - acc_data.y) < ALLOWED_ACC_ERROR) &&
	  (fabsf(old_acc_data.z - acc_data.z) < ALLOWED_ACC_ERROR) &&
	  (fabsf(old_gyro_data.x - gyro_data.x) < ALLOWED_GYRO_ERROR) &&
	  (fabsf(old_gyro_data.y - gyro_data.y) < ALLOWED_GYRO_ERROR) &&
	  (fabsf(old_gyro_data.z - gyro_data.z) < ALLOWED_GYRO_ERROR)) {
		memoryBit++;
	} else {
		memoryBit=0;
	}
    old_acc_data = acc_data;
  	old_gyro_data = gyro_data;

	if(memoryBit>TRESHOLD) {
		//param 1 -> old phase
		// parm 2 -> new phase
		// parm 3 -> events to trigger
		change_state_to( phase,READY, EV_READY);
	}



static void check_ready_to_phase(Phase *phase , Acc acc_data){
	// if I'm in a higher state I cannot come back
	if(*phase>READY) return;


	const float accel_x = acc_data.x * acc_data.x;
	const float accel_y = acc_data.y * acc_data.y;
	const float accel_z = acc_data.z * acc_data.z;
	const float acceleration = accel_x + accel_y + accel_z;
	old_acc_data = acc_data;
	old_gyro_data = gyro_data;
	// do we use? or can we use a flag instaed?
	//	state->memory[1] > LIFTOFF_SAFETY_COUNTER
	if (acceleration > ( (float) settings->liftoff_acc_threshold * (float) liftoff_acc_threshold) {
		memoryBit++;
	  } else {
		memoryBit=0;
	  }

	if (memoryBit > LIFTOFF_SAFETY_COUNTER) {
	// parm 1 -> new phase
	// parm 2 -> events to trigger
	// struct we don't use (remove?)
		change_state_to( phase,THRUSTING, EV_LIFTOFF);
	}

}
static void check_burning_phase(Phase *phase,Acc acc_data, estimation_output_t state_data){
	// if I'm in a higher state I cannot come back
	if(*phase>THRUSTING) return;

	memoryBit=0;
	 /* When acceleration is below 0, liftoff concludes */
	if (state_data.acceleration < 0) {
		memoryBit++;
	} else {
		memoryBit=0;
	}

	if (memoryBit > COASTING_SAFETY_COUNTER) {
		change_state_to(phase,COASTING, EV_MAX_V);
	}
}


static void check_coasting_phase(Phase *phase,Acc acc_data, estimation_output_t state_data){
	// if I'm in a higher state I cannot come back
	if(*phase>COASTING) return;

	memoryBit=0;
	/* When velocity is below 0, coasting concludes */
	if (state_data.velocity < 0) {
		memoryBit++;
	}

	if (memoryBit> APOGEE_SAFETY_COUNTER) {
		/* If the duration between thrusting and apogee is smaller than defined, go to touchdown */
		if ((osKernelGetTickCount() - memoryBitthrust_trigger_time) < MIN_TICK_COUNTS_BETWEEN_THRUSTING_APOGEE) {
			change_state_to(TOUCHDOWN, EV_TOUCHDOWN, fsm_state);
		} else {
			change_state_to(DROGUE, EV_APOGEE, fsm_state);
		}
	}

}
static void check_drogue_deploy_phase(Phase *phase,Acc acc_data, estimation_output_t state_data){
	// if I'm in a higher state I cannot come back
	if(*phase>DROGUE) return;

	memoryBit=0;
	if (state_data.height < static_cast<float32_t>(global_cats_config.control_settings.main_altitude)) {
		/* Achieved Height to deploy Main */
		memoryBit++;
	} else {
		/* Did Not Achieve */
		memoryBit=0;
	}

	if (memoryBit > MAIN_SAFETY_COUNTER) {
		change_state_to(phase,MAIN, EV_MAIN_DEPLOYMENT);
	}

}
static void check_main_deploy_phase(Phase *phase, Acc acc_data,estimation_output_t state_data){
	// if I'm in a higher state I cannot come back
	if(*phase>MAIN) return;

	memoryBit=0;

	/* If the velocity is very small we have touchdown */
	if (fabsf(state_data.velocity) < VELOCITY_BOUND_TOUCHDOWN) {
		/* Touchdown achieved */
		memoryBit++;
	} else {
		/* Touchdown not achieved */
		memoryBit= 0;
	}

	if (memoryBit > TOUCHDOWN_SAFETY_COUNTER) {
		change_state_to(phase,TOUCHDOWN, EV_TOUCHDOWN);
	}

}
static void check_touch_down_phase(Phase *phase, Acc acc_data,estimation_output_t state_data){
	// if I'm in a higher state I cannot come back
	if(*phase>TOUCHDOWN) return;

	memoryBit=0;

	if ((fabsf(old_acc_data.x - acc_data.x) < ALLOWED_ACC_ERROR) &&
	  (fabsf(old_acc_data.y - acc_data.y) < ALLOWED_ACC_ERROR) &&
	  (fabsf(old_acc_data.z - acc_data.z) < ALLOWED_ACC_ERROR)) {
		/* Touchdown achieved */
		memoryBit++;
	} else {
		/* Touchdown not achieved */
		memoryBit= 0;
	}

	if (memoryBit > TOUCHDOWN_SAFETY_COUNTER) {
		change_state_to(phase,GROUND, EV_GROUND);
	}
}

static void clear_memory_bit(uint16_t *value){
	*value=0;
}


static void change_state_to(Phase *old_phase,Phase new_phase, Events_trigger event_to_trigger) {
	if (flight_state == THRUSTING) {
		thrust_trigger_time = osKernelGetTickCount();
	}

	trigger_event(event_to_trigger);
	osEventFlagsClear(fsm_flag_id, 0xFF);
	osEventFlagsSet(fsm_flag_id, new_state);
	//go to
	*old_phase=new_phase;
	clear_memory_bit(&memoryBit);
}

#endif
