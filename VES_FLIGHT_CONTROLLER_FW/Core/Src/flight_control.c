/*
 * FlightControl.c
 *
 *  Created on: Apr 2, 2024
 *      Author: Tommaso & Francesco Abate
 */

#ifndef FLIGHTCONTROL_H
#include "flight_control.h"
#endif

float_t previous_altitude = 0;

static void clear_fsm_memory(flight_fsm_t *fsm_state);

osStatus_t trigger_event(events_trigger ev, bool event_unique) {
    static uint32_t event_tracking = 0U;

    if (event_unique) {
        if (event_tracking & (1U << ev)) {
            return osOK;
        }

        if (ev == EV_TOUCHDOWN) {
            event_tracking = 0xFFFFFFFF;
        }
    }

//    log_warn("Event %lu Queued", ev);
    // TODO: check if timeout should be 0 here
    return osMessageQueuePut(event_queue, &ev, 0U, 10U);
}

void check_flight_phase(flight_fsm_t *phase, estimation_output_t MotionData,linear_acceleration_t acc_data, linear_acceleration_t gyro_data) {

	 const flight_fsm_t old_phase = *phase;

	 switch (phase->flight_state) {

	 case CALIBRATING:
		 	 check_Calibrating_phase(phase,MotionData,acc_data,gyro_data);
		 	 break;

	    case READY:
			  check_Ready_phase(phase, MotionData,acc_data);
			  break;

	    case BURNING:
			  check_Burning_phase(phase, MotionData);
			  break;

	    case COASTING:
	    		check_Coasting_phase(phase,MotionData);
	    		break;

	    case DROGUE:
	    	  check_Drogue_phase(phase,MotionData);
			  break;

	    case MAIN:
			  check_Main_phase(phase, MotionData);
			  break;

	    case TOUCHDOWN:
			  check_Touchdown_phase(phase, MotionData,acc_data);
			  break;
	    case INVALID:
	    	  break;

	  }

	 phase->state_changed = old_phase.flight_state != phase->flight_state;
}

void check_Calibrating_phase(flight_fsm_t *fsm_state,estimation_output_t MotionData,linear_acceleration_t acc_data, linear_acceleration_t gyro_data){
	if(fsm_state->flight_state > CALIBRATING) return;

	if ((fabsf(fsm_state->old_acc_data.accX - acc_data.accX) < ACCELEROMETER_MEASURE_TOLERANCE_MG) &&
		  (fabsf(fsm_state->old_acc_data.accY - acc_data.accY) < ACCELEROMETER_MEASURE_TOLERANCE_MG) &&
		  (fabsf(fsm_state->old_acc_data.accZ - acc_data.accZ) < ACCELEROMETER_MEASURE_TOLERANCE_MG) &&
		  (fabsf(fsm_state->old_gyro_data.accX - gyro_data.accX) < GYRO_MEASURE_TOLERANCE_MDPS) &&
		  (fabsf(fsm_state->old_gyro_data.accY - gyro_data.accY) < GYRO_MEASURE_TOLERANCE_MDPS) &&
		  (fabsf(fsm_state->old_gyro_data.accZ - gyro_data.accZ) < GYRO_MEASURE_TOLERANCE_MDPS)) {

			fsm_state->memory[0]++;
		} else {

			fsm_state->memory[0] = 0;
		}

		fsm_state->old_acc_data = acc_data;
		fsm_state->old_gyro_data = gyro_data;

		if(fsm_state->memory[0] > THRESHOLD) {
			change_state_to(fsm_state, READY, EV_READY);
		}
}

void check_Ready_phase(flight_fsm_t *fsm_state,estimation_output_t MotionData, linear_acceleration_t acc_data) {
	// if I'm in a higher state I cannot come back
	if(fsm_state->flight_state > READY) return;

	float acc = sqrt(acc_data.accX * acc_data.accX + acc_data.accY * acc_data.accY + acc_data.accZ * acc_data.accZ);

	if (acc>LIFT_OFF_ACC_THRESHOLD) {

		fsm_state->memory[0]++;
	} else {

		fsm_state->memory[0] = 0;
	}

	if(fsm_state->memory[0] > AIRBRAKES_SAFETY_COUNTER) {
		change_state_to(fsm_state, BURNING, EV_BURNING);
	}

}

void check_Burning_phase(flight_fsm_t *phase, estimation_output_t MotionData) {
	// if I'm in a higher state I cannot come back

	if(phase->flight_state > BURNING) return;

	if (MotionData.acceleration < 0) {
		phase->memory[0]++;
	  } else {
		  phase->memory[0] = 0;
	  }

	if (phase->memory[0] > COASTING_SAFETY_COUNTER) {
		change_state_to(phase, COASTING, EV_COASTING);
	}

}

void check_Coasting_phase(flight_fsm_t *phase, estimation_output_t MotionData){
	if(phase->flight_state > COASTING) return;

	if (MotionData.height <= previous_altitude) {
			phase->memory[0]++;
		}
	else{
		phase->memory[0] = 0;
	}

	previous_altitude = MotionData.height;



	if(phase->memory[0] > APOGEE_SAFETY_COUNTER){
		if ((osKernelGetTickCount() - phase->thrust_trigger_time) < MIN_TICK_COUNTS_BETWEEN_BURNING_APOGEE) {
					change_state_to(phase,TOUCHDOWN, EV_TOUCHDOWN);
		} else {
					change_state_to(phase,DROGUE, EV_DROGUE);
		}
	}
}


void check_Drogue_phase(flight_fsm_t *phase, estimation_output_t MotionData) {
	// if I'm in a higher state I cannot come back
	if(phase->flight_state > DROGUE) return;


	if(MotionData.height<MAIN_DEPLOY_HEIGHT){
		phase->memory[0]++;
	}else{
		phase->memory[0] = 0;
	}



	if (phase->memory[0] > MAIN_SAFETY_COUNTER) {
		change_state_to(phase, MAIN, EV_MAIN);
	}
}

void check_Main_phase(flight_fsm_t *phase, estimation_output_t MotionData) {
	// if I'm in a higher state I cannot come back
	if(phase->flight_state > MAIN) return;

	/* If the velocity is very small we have touchdown */
	// check the altitude for a specific amount of time
	if (fabsf(MotionData.height) < 5) {
//		/* Touchdown achieved */
		phase->memory[0]++;
	} else {
		/* Touchdown not achieved */
		phase->memory[0] = 0;
	}

	if (phase->memory[0] > TOUCHDOWN_SAFETY_COUNTER) {
		change_state_to(phase, TOUCHDOWN, EV_TOUCHDOWN);
	}

}

void check_Touchdown_phase(flight_fsm_t *fsm_state,estimation_output_t MotionData, linear_acceleration_t acc_data) {
	// if I'm in a higher state I cannot come back
	if(fsm_state->flight_state > TOUCHDOWN) return;

	if ((fabsf(fsm_state->old_acc_data.accX - acc_data.accX ) < ACCELEROMETER_MEASURE_TOLERANCE_MG) &&
			(fabsf(fsm_state->old_acc_data.accY - acc_data.accY) < ACCELEROMETER_MEASURE_TOLERANCE_MG) &&
			(fabsf(fsm_state->old_acc_data.accZ - acc_data.accZ) < ACCELEROMETER_MEASURE_TOLERANCE_MG)) {
			/* Touchdown achieved */
			fsm_state->memory[0]++;
		} else {
			/* Touchdown not achieved */
			fsm_state->memory[0] = 0;
		}

	fsm_state->old_acc_data = acc_data;


	if (fsm_state->memory[0] > TOUCHDOWN_SAFETY_COUNTER) {
		change_state_to(fsm_state, TOUCHDOWN, EV_TOUCHDOWN);


	}

}





/* Function that needs to be called every time that a state transition is done */
static void clear_fsm_memory(flight_fsm_t *phase) {
	phase->clock_memory = 0;
	phase->memory[0] = 0;
	phase->memory[1] = 0;
	phase->memory[2] = 0;
}

 void change_state_to(flight_fsm_t *phase, flight_phase_t new_phase, events_trigger event_to_trigger) {


	trigger_event(event_to_trigger,true);
	osEventFlagsClear(fsm_flag_id, 0xFF);
	osEventFlagsSet(fsm_flag_id, new_phase);
	//go to
	phase->flight_state=new_phase;
	clear_fsm_memory(phase);
}
