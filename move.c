#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <motors.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <sensors/proximity.h>
#include <move.h>
#include <audio_processing.h>

#define DEFAULT_SPEED			0.7 * MOTOR_SPEED_LIMIT // step/s

#define INSTRUCTION_OBS_DIST	150 // mm

static int16_t leftSpeed = 0, rightSpeed = 0;
static bool done=true;


//returns true if an obstacle is 15 cm away
bool obstacle_detected(void){
	return (VL53L0X_get_dist_mm()>=(INSTRUCTION_OBS_DIST-10))
			&& (VL53L0X_get_dist_mm()<=(INSTRUCTION_OBS_DIST+10));
}

bool end_left_wall(void){
	return ((get_calibrated_prox(IR6)>50)&&(get_calibrated_prox(IR6)<100)&&(get_calibrated_prox(IR7)<15));
}

bool end_right_wall(void){
	return ((get_calibrated_prox(IR3)>200)&&(get_calibrated_prox(IR3)<300)&&(get_calibrated_prox(IR2)<15));
}


void rotate_right(void){

	leftSpeed=0.5*MOTOR_SPEED_LIMIT;
	rightSpeed =0.5*MOTOR_SPEED_LIMIT - 1*get_calibrated_prox(IR8) - 0.5*get_calibrated_prox(IR7);

	left_motor_set_speed(leftSpeed);
	right_motor_set_speed(rightSpeed);

	if (end_left_wall()){
		done=true;
	}

}

void rotate_left(void){


	leftSpeed=0.5*MOTOR_SPEED_LIMIT- 1*get_calibrated_prox(IR1) - 0.5*get_calibrated_prox(IR2);
	rightSpeed =0.5*MOTOR_SPEED_LIMIT;

	left_motor_set_speed(leftSpeed);
	right_motor_set_speed(rightSpeed);

	if (end_right_wall()){
			done=true;
		}

}

void rond_point(void){
	static uint8_t nb_turn=0;
	static bool turningleft=0;

	if(nb_turn==0){
		leftSpeed= 0.45*MOTOR_SPEED_LIMIT;
		rightSpeed =0.5*MOTOR_SPEED_LIMIT-1*get_calibrated_prox(IR8) - 0.5*get_calibrated_prox(IR7);
	}
	if( end_left_wall() && (!turningleft))
	{
			if(nb_turn==2){
			done=1;
			nb_turn=0,turningleft=false;
			}else{
			turningleft=true;
			}
	}
	if(turningleft){
		leftSpeed=0.1*MOTOR_SPEED_LIMIT;
		rightSpeed =0.5*MOTOR_SPEED_LIMIT;
		if(get_calibrated_prox(IR7)>600){
			nb_turn++;
			turningleft=false;
		}
	}else if((!turningleft)&&(nb_turn>=1)){
		leftSpeed= 0.45*MOTOR_SPEED_LIMIT;
		rightSpeed =0.5*MOTOR_SPEED_LIMIT-1*get_calibrated_prox(IR8) - 0.5*get_calibrated_prox(IR7);
	}

}

void send_to_computer(TO_DO instruction){
	static uint8_t mustSend = 0;
	if(mustSend > 8){
	chprintf((BaseSequentialStream *)&SD3, "INSTRUCTION= %d,distance= %d \r\n",instruction,VL53L0X_get_dist_mm());
	chprintf((BaseSequentialStream *)&SD3, "speed\r\n");
	chprintf((BaseSequentialStream *)&SD3, "%4d,%4d,\r\n\n",leftSpeed,rightSpeed);
	chprintf((BaseSequentialStream *)&SD3, "Calibrated\r\n");
	chprintf((BaseSequentialStream *)&SD3, "%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d\r\n\n", get_calibrated_prox(IR1), get_calibrated_prox(IR2),get_calibrated_prox(IR3),get_calibrated_prox(IR4), get_calibrated_prox(IR5), get_calibrated_prox(IR6), get_calibrated_prox(IR7), get_calibrated_prox(IR8));
	mustSend = 0;
	}
	mustSend++;
}

static THD_WORKING_AREA(waMovement, 1024);
static THD_FUNCTION(Movement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    static TO_DO instruction=0;

    //waits until a start signal (FREQ_START sound) is detected
	wait_start_signal();


    while(1){
    	time = chVTGetSystemTime();
/*
		if (get_next_instruction()==START){
			chprintf((BaseSequentialStream *)&SD3, "START %f  \n", get_norm());
			left_motor_set_speed(DEFAULT_SPEED);
			right_motor_set_speed(DEFAULT_SPEED);
		 }
*/

    	if (obstacle_detected()&& done){
			instruction=get_next_instruction();
			done=false;
		}
		if (!done){
			switch (instruction)
			{
				case TURN_RIGHT:
					rotate_right();
					break;
				case TURN_LEFT:
					rotate_left();
					break;
				case RONDPOINT:
					rond_point();
					break;
				case PARK:
					break;
				case STOP:
					left_motor_set_speed(0);
					right_motor_set_speed(0);
					break;
				default:
					left_motor_set_speed(0);
					right_motor_set_speed(0);
					break;
			}


		}
		if (done){
			leftSpeed=DEFAULT_SPEED;
			rightSpeed =DEFAULT_SPEED;
		}

    	send_to_computer(instruction);
    	left_motor_set_speed(leftSpeed);
    	right_motor_set_speed(rightSpeed);
    	chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz
    }
 }

void movement_start(void){
	chThdCreateStatic(waMovement, sizeof(waMovement), NORMALPRIO, Movement, NULL);
}
