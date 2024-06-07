/*
 * FlightControl.c
 *
 *  Created on: Apr 2, 2024
 *      Author: tommaso
 * 		Contributor: Vincenzo Catalano
 */

#ifndef FLIGHTCONTROL_H
#include "FlightControl.h"

uint16_t memoryBit = 0;
const float ACC_SENSOR_ERROR = 0;
const float ALLOWED_ACC_ERROR = 0.1F; // Assegna un valore di esempio
const float ALLOWED_GYRO_ERROR = 0.1F; // Assegna un valore di esempio
const float MAIN_SAFETY_COUNTER = 5.0F; // Assegna un valore di esempio
const float COASTING_SAFETY_COUNTER = 5.0F; // Assegna un valore di esempio
const uint16_t LIFTOFF_SAFETY_COUNTER = 10; // Assegna un valore di esempio
const uint16_t TOUCHDOWN_SAFETY_COUNTER = 100;
const float VELOCITY_BOUND_TOUCHDOWN = 3.0F;
const uint16_t APOGEE_SAFETY_COUNTER = 30;
const uint16_t TRESHOLD = 10;
const float LIFT_OFF_ACC_TRESHOLD = 1.5F; // Assegna un valore di esempio

void check_flight_phase(Phase *phase, Acc acc_data, Acc old_acc_data, Acc gyro_data,Acc old_gyro_data,Vel vel){
	 switch (*phase) {
	    case CALIBRATING:
			  check_system_calibrating_phase( phase, acc_data,old_acc_data, gyro_data,old_gyro_data);
			  break;
	    case READY:
			  check_ready_takeoff_phase( phase, acc_data);
			  break;
	    case BURNING:
			  check_burning_phase( phase, acc_data);
			  break;
	    case COASTING:
			  check_coasting_phase( phase, acc_data);
			  break;
	    case DROGUE:
			  check_drogue_deploy_phase( phase, acc_data);
			  break;
	    case MAIN:
			  check_main_deploy_phase( phase, acc_data,vel);
			  break;
	    case TOUCHDOWN:
	    		check_touchdown_phase( phase, acc_data);
	    	break;
	    case INVALID:
	    	break;

	  }
}

void check_system_calibrating_phase(Phase *phase , Acc acc_data, Acc old_acc_data, Acc gyro_data, Acc old_gyro_data){
	// if I'm in a higher state I cannot come back
	if(*phase>CALIBRATING) return;

	if ((fabsf(old_acc_data.acc1 - acc_data.acc1) < ALLOWED_ACC_ERROR) &&
	  (fabsf(old_acc_data.acc2- acc_data.acc2) < ALLOWED_ACC_ERROR) &&
	  (fabsf(old_acc_data.acc3 - acc_data.acc3) < ALLOWED_ACC_ERROR) &&
	  (fabsf(old_gyro_data.acc1- gyro_data.acc1) < ALLOWED_GYRO_ERROR) &&
	  (fabsf(old_gyro_data.acc2 - gyro_data.acc3) < ALLOWED_GYRO_ERROR) &&
	  (fabsf(old_gyro_data.acc3- gyro_data.acc3) < ALLOWED_GYRO_ERROR)) {
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

}

void check_ready_to_phase(Phase *phase , Acc acc_data){
	// if I'm in a higher state I cannot come back
	if(*phase>READY) return;


	const float accel_x = acc_data.acc1 * acc_data.acc1;
	const float accel_y = acc_data.acc2* acc_data.acc2;
	const float accel_z = acc_data.acc3* acc_data.acc3;
	const float acceleration = accel_x + accel_y + accel_z;

	// do we use? or can we use a flag instaed?
	//	state->memory[1] > LIFTOFF_SAFETY_COUNTER
	if (acceleration >  LIFT_OFF_ACC_TRESHOLD) {
		memoryBit++;
	  } else {
		memoryBit=0;
	  }

	if (memoryBit > LIFTOFF_SAFETY_COUNTER) {
	// parm 1 -> new phase
	// parm 2 -> events to trigger
	// struct we don't use (remove?)
		change_state_to( phase,BURNING, EV_LIFTOFF);
	}

}
void check_burning_phase(Phase *phase,Acc acc_data){
	// if I'm in a higher state I cannot come back
	if(*phase>BURNING) return;

	memoryBit=0;
	 /* When acceleration is below 0, liftoff concludes */
	if (acc_data.acc3 > LIFT_OFF_ACC_TRESHOLD) {
		memoryBit++;
	} else {
		memoryBit=0;
	}

	if (memoryBit > COASTING_SAFETY_COUNTER) {
		change_state_to(phase,COASTING, EV_MAX_V);
	}
}


void check_coasting_phase(Phase *phase,Acc acc_data){
	// if I'm in a higher state I cannot come back
	if(*phase>COASTING) return;

	memoryBit=0;
	/* When velocity is below 0, coasting concludes */
	if (acc_data.acc3 < 0) {
		memoryBit++;
	}

	if (memoryBit> APOGEE_SAFETY_COUNTER) {
		/* If the duration between BURNING and apogee is smaller than defined, go to touchdown */
	/*	if ((osKernelGetTickCount() - memoryBitthrust_trigger_time) < MIN_TICK_COUNTS_BETWEEN_BURNING_APOGEE) {
			change_state_to(TOUCHDOWN, EV_TOUCHDOWN, fsm_state);
		} else {
			change_state_to(DROGUE, EV_APOGEE, fsm_state);
		}*/
	}

}
void check_drogue_deploy_phase(Phase *phase,Acc acc_data){
	// if I'm in a higher state I cannot come back
	if(*phase>DROGUE) return;

	memoryBit=0;
	//CONDIZIONE di verifica del diploy
	if (1) {
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
void check_main_deploy_phase(Phase *phase, Acc acc_data,Vel vel_data){
	// if I'm in a higher state I cannot come back
	if(*phase>MAIN) return;

	memoryBit=0;

	/* If the velocity is very small we have touchdown */
	if (vel_data.vel < VELOCITY_BOUND_TOUCHDOWN) {
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
void check_touch_down_phase(Phase *phase, Acc acc_data){
	// if I'm in a higher state I cannot come back
	if(*phase>TOUCHDOWN) return;

	memoryBit=0;

	if ((fabsf(acc_data.acc1 ) < ALLOWED_ACC_ERROR) &&
	  (fabsf(acc_data.acc2) < ALLOWED_ACC_ERROR) &&
	  (fabsf(acc_data.acc3) < ALLOWED_ACC_ERROR)) {
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

void clear_memory_bit(uint16_t *value){
	*value=0;
}


 void change_state_to(Phase *old_phase,Phase new_phase, Events_trigger event_to_trigger) {
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
#endif
