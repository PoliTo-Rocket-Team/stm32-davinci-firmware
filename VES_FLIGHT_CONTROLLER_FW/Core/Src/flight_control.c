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

        if (ev == EV_TERRA) {
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

	    case TERRA:
			  check_Terra_phase(phase, MotionData,acc_data, gyro_data);
			  break;

	    case AEREO:
			  check_Aereo_phase(phase, MotionData ,acc_data);
			  break;

	    case CADUTA:
	    		check_Caduta_phase(phase,MotionData);
	    		break;

	    case UN_QUARTO:
	    	  check_Un_Quarto_phase(phase,MotionData);
			  break;

	    case MID:
			  check_Mid_phase(phase, MotionData);
			  break;

	    case TRE_QUARTI:
			  check_Tre_Quarti_phase(phase, MotionData);
			  break;

	  }

	 phase->state_changed = old_phase.flight_state != phase->flight_state;
}

void check_Terra_phase(flight_fsm_t *phase, estimation_output_t MotionData,linear_acceleration_t acc_data, linear_acceleration_t gyro_data) {
	// if I'm in a higher state I cannot come back
	if(phase->flight_state > TERRA) return;

	if (MotionData.height<800) {

		phase->memory[0]++;
	} else {

		phase->memory[0] = 0;
	}

	if(phase->memory[0] > THRESHOLD) {
		change_state_to(phase, AEREO, EV_AEREO);
	}

}

void check_Aereo_phase(flight_fsm_t *phase, estimation_output_t MotionData,linear_acceleration_t acc_data) {
	// if I'm in a higher state I cannot come back

	if(phase->flight_state > AEREO) return;

	if (MotionData.height <= previous_altitude) {
		phase->memory[0]++;
	}
	previous_altitude = MotionData.height;

	if (phase->memory[0] >  NUMBER_OF_CADUTE) {
		phase->memory[1]++;
	  } else {
		  phase->memory[1] = 0;
	  }

	if (phase->memory[1] > CADUTE_SAFETY_COUNTER) {
		change_state_to(phase, CADUTA, EV_CADUTA);
	}

}

void check_Caduta_phase(flight_fsm_t *phase, estimation_output_t MotionData){
	if(phase->flight_state > CADUTA) return;

	if(MotionData.height< ALTITUDE_QUARTO){
		phase->memory[0]++;
	}else{
		phase->memory[0] = 0;
	}

	if(phase->memory[0] > QUARTO_SAFETY_COUNTER){
		change_state_to(phase,UN_QUARTO,EV_UN_QUARTO);
		W25Q128_t *flash;
		flash = get_flash();
		uint8_t address[3] = {0, 0, 0}; // Address of the first byte
		uint8_t data_flown[1];
		W25Q128_write_flown_flag(flash, address, data_flown, 1,1);
//		W25Q128_read_flown_flag(flash, address, data_flown, 1);
	}
}


void check_Un_Quarto_phase(flight_fsm_t *phase, estimation_output_t MotionData) {
	// if I'm in a higher state I cannot come back
	if(phase->flight_state > UN_QUARTO) return;


	if(MotionData.height< ALTITUDE_MID){
		phase->memory[0]++;
	}else{
		phase->memory[0] = 0;
	}



	if (phase->memory[0] > MID_SAFETY_COUNTER) {
		change_state_to(phase, MID, EV_TRE_QUARTI);
	}
}

void check_Mid_phase(flight_fsm_t *phase, estimation_output_t MotionData) {
	// if I'm in a higher state I cannot come back
	if(phase->flight_state > MID) return;
	/* When velocity is below 0, coasting concludes */
	// if we need to check acceleration be sure that accZ is the correct one to check
	//XXX check the altitude for a specific amount of time

	if (MotionData.height < ALTITUDE_TRE_QUARTI) {
		phase->memory[0]++;
	}else{
		phase->memory[0] = 0;
	}

	if (phase->memory[0] > TRE_QUARTI_SAFETY_COUNTER) {
		/* If the duration between BURNING and apogee is smaller than defined, go to touchdown */
			change_state_to(phase,TRE_QUARTI, EV_TRE_QUARTI);

	}

}

void check_Tre_Quarti_phase(flight_fsm_t *phase, estimation_output_t MotionData) {
	// if I'm in a higher state I cannot come back
	if(phase->flight_state > TRE_QUARTI) return;

	if(MotionData.height < ALTITUDE_ATTERRAGGIO){
		/* Achieved Height to deploy Main */

		phase->memory[0]++;
	} else {
		/* Did Not Achieve */
		phase->memory[0]= 0;
	}

	if (phase->memory[0] > ATTERRAGGIO_SAFETY_COUNTER) {//PER ORA 5 MA SI PUO CAMBIARE
		change_state_to(phase,TERRA, EV_TERRA);
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
