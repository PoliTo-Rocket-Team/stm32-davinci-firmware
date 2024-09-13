/*
 * FlightControl.c
 *
 *  Created on: Apr 2, 2024
 *      Author: Tommaso & Francesco Abate
 */

#ifndef FLIGHTCONTROL_H
#include "flight_control.h"
#endif

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
			  check_system_calibrating_phase(phase, MotionData,acc_data, gyro_data);
			  break;

	    case READY:
			  check_ready_to_takeoff_phase(phase, MotionData ,acc_data);
			  break;

	    case BURNING:
			  check_burning_phase(phase, MotionData);
			  break;

	    case COASTING:
			  check_coasting_phase(phase, MotionData);
			  break;

	    case DROGUE:
			  check_drogue_deploy_phase(phase, MotionData);
			  break;

	    case MAIN:
			  check_main_deploy_phase(phase, MotionData);
			  break;

	    case TOUCHDOWN:
	    	check_touch_down_phase(phase, MotionData,acc_data);
	    	break;

	    case INVALID:
	    	break;

	  }

	 phase->state_changed = old_phase.flight_state != phase->flight_state;
}

void check_system_calibrating_phase(flight_fsm_t *phase, estimation_output_t MotionData,linear_acceleration_t acc_data, linear_acceleration_t gyro_data) {
	// if I'm in a higher state I cannot come back
	if(phase->flight_state > CALIBRATING) return;

	if ((fabsf(phase->old_acc_data.accX - acc_data.accX) < ACCELEROMETER_MEASURE_TOLERANCE_MG) &&
	  (fabsf(phase->old_acc_data.accY - acc_data.accY) < ACCELEROMETER_MEASURE_TOLERANCE_MG) &&
	  (fabsf(phase->old_acc_data.accZ - acc_data.accZ) < ACCELEROMETER_MEASURE_TOLERANCE_MG) &&
	  (fabsf(phase->old_gyro_data.accX - gyro_data.accX) < GYRO_MEASURE_TOLERANCE_MDPS) &&
	  (fabsf(phase->old_gyro_data.accY - gyro_data.accY) < GYRO_MEASURE_TOLERANCE_MDPS) &&
	  (fabsf(phase->old_gyro_data.accZ - gyro_data.accZ) < GYRO_MEASURE_TOLERANCE_MDPS)) {

		phase->memory[0]++;
	} else {

		phase->memory[0] = 0;
	}

    phase->old_acc_data = acc_data;
    phase->old_gyro_data = gyro_data;

	if(phase->memory[0] > THRESHOLD) {
		change_state_to(phase, READY, EV_READY);
	}

}

void check_ready_to_takeoff_phase(flight_fsm_t *phase, estimation_output_t MotionData,linear_acceleration_t acc_data) {
	// if I'm in a higher state I cannot come back

	if(phase->flight_state > READY) return;

	const float accel_x = acc_data.accX * acc_data.accX;
	const float accel_y = acc_data.accY * acc_data.accY;
	const float accel_z = acc_data.accZ * acc_data.accZ;
	const float acceleration = accel_x + accel_y + accel_z;

	// do we use? or can we use a flag instead?
	//	state->memory[1] > LIFTOFF_SAFETY_COUNTER
	if (acceleration >  LIFT_OFF_ACC_THRESHOLD) {
		phase->memory[1]++;
	  } else {
		  phase->memory[1] = 0;
	  }

	if (phase->memory[1] > LIFTOFF_SAFETY_COUNTER) {
		change_state_to(phase, BURNING, EV_LIFTOFF);
	}

}

void check_burning_phase(flight_fsm_t *phase, estimation_output_t MotionData) {
	// if I'm in a higher state I cannot come back
	if(phase->flight_state > BURNING) return;


	if(MotionData.acceleration< 0 ){
		phase->memory[0]++;
	}else{
		phase->memory[0] = 0;
	}


	if (phase->memory[0] > COASTING_SAFETY_COUNTER) {
		change_state_to(phase, COASTING, EV_MAX_V);
	}
}

void check_coasting_phase(flight_fsm_t *phase, estimation_output_t MotionData) {
	// if I'm in a higher state I cannot come back
	if(phase->flight_state > COASTING) return;
	/* When velocity is below 0, coasting concludes */
	// if we need to check acceleration be sure that accZ is the correct one to check
	//XXX check the altitude for a specific amount of time
	if (MotionData.height>650) {
		phase->memory[0]++;
	}

	if (phase->memory[0] > APOGEE_SAFETY_COUNTER) {
		/* If the duration between BURNING and apogee is smaller than defined, go to touchdown */
		if ((osKernelGetTickCount() - phase->thrust_trigger_time) < MIN_TICK_COUNTS_BETWEEN_BURNING_APOGEE) {
			change_state_to(phase,TOUCHDOWN, EV_TOUCHDOWN);
		} else {
			change_state_to(phase,DROGUE, EV_APOGEE);
		}
	}

}

void check_drogue_deploy_phase(flight_fsm_t *phase, estimation_output_t MotionData) {
	// if I'm in a higher state I cannot come back
	if(phase->flight_state > DROGUE) return;
	/////////////////////////////////////////////////////////////////////////////
	if(MotionData.height<MAIN_DEPLOY_HEIGHT){//TO BE CHANGED : Apogee height at which we want to open our parachutes.
		/* check the current altitude in order to decide whether to deploy the drogue or not */
	/////////////////////////////////////////////////////////////////////////////
		/* Achieved Height to deploy Main */

		phase->memory[0]++;
	} else {
		/* Did Not Achieve */
		phase->memory[0]= 0;
	}

	if (phase->memory[0] > MAIN_SAFETY_COUNTER) {//PER ORA 5 MA SI PUO CAMBIARE
		change_state_to(phase,MAIN, EV_MAIN_DEPLOYMENT);
	}

}

void check_main_deploy_phase(flight_fsm_t *phase, estimation_output_t MotionData) {
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

void check_touch_down_phase(flight_fsm_t *phase, estimation_output_t MotionData, linear_acceleration_t acc_data) {
	// if I'm in a higher state I cannot come back
	if(phase->flight_state > TOUCHDOWN) return;



	if ((fabsf(phase->old_acc_data.accX - acc_data.accX ) < ACCELEROMETER_MEASURE_TOLERANCE_MG) &&
		(fabsf(phase->old_acc_data.accY - acc_data.accY) < ACCELEROMETER_MEASURE_TOLERANCE_MG) &&
		(fabsf(phase->old_acc_data.accZ - acc_data.accZ) < ACCELEROMETER_MEASURE_TOLERANCE_MG)) {
		/* Touchdown achieved */
		phase->memory[0]++;
	} else {
		/* Touchdown not achieved */
		phase->memory[0] = 0;
	}

	if (phase->memory[0] > TOUCHDOWN_SAFETY_COUNTER) {
		change_state_to(phase, TOUCHDOWN, EV_TOUCHDOWN);
		W25Q128_t *flash;
		flash = get_flash();
		uint8_t address[3] = {0, 0, 0}; // Address of the first byte
		uint8_t data_flown[1];
		W25Q128_write_flown_flag(flash, address, data_flown, 1,1);


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
	if (phase->flight_state == BURNING) {
		phase->thrust_trigger_time = osKernelGetTickCount();
	}

	trigger_event(event_to_trigger,true);
	osEventFlagsClear(fsm_flag_id, 0xFF);
	osEventFlagsSet(fsm_flag_id, new_phase);
	//go to
	phase->flight_state=new_phase;
	clear_fsm_memory(phase);
}
