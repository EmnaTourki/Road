#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <motors.h>
#include <leds.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <sensors/proximity.h>
#include <move.h>
#include <audio_processing.h>
#include <audio/play_melody.h>
#include <audio/audio_thread.h>



#define PI                  	3.1415926536f
#define WHEEL_DISTANCE      	5.30f   					//cm
#define PERIMETER_EPUCK     	(PI * WHEEL_DISTANCE)		//cm
#define NSTEP_ONE_TURN      	1000 						// number of step for 1 turn of the motor
#define WHEEL_PERIMETER			13 							// cm
#define POSITION_ROTATION_90 	(0.25*PERIMETER_EPUCK) * NSTEP_ONE_TURN / WHEEL_PERIMETER //step
#define CORRECTION_ROTATION_90 	1
#define DEFAULT_SPEED			0.6 * MOTOR_SPEED_LIMIT 	// step/s
#define OBS_DIST_15cm			130 						// mm
#define OBS_DIST_38cm			380							// mm
#define THRESHOLD_15cm			20							// mm
#define THRESHOLD_38cm			80							// mm
#define PLACE_DIM_MIN 			480							// step
#define MAX_NB_INSTRUCTION		20

static int16_t leftSpeed = 0, rightSpeed = 0;
static bool done=true;
static bool parkdone=false;
static int32_t to_computer_dim=0;


//returns true if an obstacle is obs_dist mm away
bool obstacle_detected(int16_t obs_dist,uint8_t threshold ){
	return (VL53L0X_get_dist_mm()>=(obs_dist-threshold))
			&& (VL53L0X_get_dist_mm()<=(obs_dist+threshold));
}


bool end_left_wall(void){
	static bool walldetected=0;
	if (get_calibrated_prox(IR6)>500){
		walldetected=true;
	}
	if	((walldetected)&&(get_calibrated_prox(IR6)<200)){
		walldetected=false;
		return true;
	}else{
			return false;
	}

}

bool end_right_wall(void){
	static bool walldetected=0;
	if (get_calibrated_prox(IR3)>500){
		walldetected=true;
	}
	if	((walldetected)&&(get_calibrated_prox(IR3)<200)){
		walldetected=false;
		return true;
	}else{
			return false;
	}

}

void clignotant(rgb_led_name_t led_1,rgb_led_name_t led_2){
	static uint8_t counter=0;
	if(counter>10){
		toggle_rgb_led(led_1, GREEN_LED, 255);
		toggle_rgb_led(led_2, GREEN_LED, 255);
		counter=0;
	}
	counter++;
}


void rotate_right(void){
	clignotant(LED2,LED4);
	leftSpeed=0.5*MOTOR_SPEED_LIMIT;
	rightSpeed =0.5*MOTOR_SPEED_LIMIT - 1.4*get_calibrated_prox(IR8) - 0.42*get_calibrated_prox(IR7);

	if (end_left_wall()){
		done=true;
	}

}

void rotate_left(void){
	clignotant(LED8,LED6);
	leftSpeed=0.5*MOTOR_SPEED_LIMIT- 1.4*get_calibrated_prox(IR1) - 0.42*get_calibrated_prox(IR2);
	rightSpeed =0.5*MOTOR_SPEED_LIMIT;

	if (end_right_wall()){
			done=true;
		}

}

void sortie_rondpoint(int32_t position_to_reach){
	static bool reset_position=0;

	clignotant(LED2,LED4);

	if(!reset_position){
		left_motor_set_pos(0);
		right_motor_set_pos(0);
		reset_position=true;
	}else if((reset_position)&&(left_motor_get_pos()>(position_to_reach))){
		leftSpeed= 0.5*MOTOR_SPEED_LIMIT;
		rightSpeed =-0.5*MOTOR_SPEED_LIMIT;

		if((left_motor_get_pos()>=(position_to_reach+CORRECTION_ROTATION_90*POSITION_ROTATION_90))){
			done=1;
			reset_position=false;
		}
	}
}

void rond_point(uint8_t exit){
	static uint8_t nb_turn=0;
	static bool turningleft=0;
	static int32_t debut=0 ,fin=0,position_to_reach=1000;
	if(nb_turn==0){
		leftSpeed= 0.45*MOTOR_SPEED_LIMIT;
		rightSpeed =0.5*MOTOR_SPEED_LIMIT-0.7*get_calibrated_prox(IR8) - 0.25*get_calibrated_prox(IR7);
	}
	if( (!turningleft)){
		if( end_left_wall() && (nb_turn!=exit)){
			turningleft=true;
		}
		if (nb_turn>=1){
			leftSpeed= 0.45*MOTOR_SPEED_LIMIT;
			rightSpeed =0.5*MOTOR_SPEED_LIMIT-0.5*get_calibrated_prox(IR8)- 0.25*get_calibrated_prox(IR7);
			if((nb_turn==1)&&(!debut)){
				debut=left_motor_get_pos();
			}
			if((nb_turn==2)&&(debut)&&(!fin)){
				fin=left_motor_get_pos();
				position_to_reach=0.5*(fin-debut);
			}
			if((nb_turn==exit)){
				sortie_rondpoint(position_to_reach);
				if (done){
					nb_turn=0,debut=0,fin=0,turningleft=false;
				}
			}

		}
	}
	else if(turningleft){
		leftSpeed=0.105*MOTOR_SPEED_LIMIT;
		rightSpeed =0.5*MOTOR_SPEED_LIMIT;
		if(get_calibrated_prox(IR7)>500){
			turningleft=false;
			nb_turn++;
		}
	}
}

bool find_a_place(void){
	static int32_t debut=0 , fin=0, empty_space_dimension=-1;
	leftSpeed=0.5*MOTOR_SPEED_LIMIT-0.5*get_calibrated_prox(IR1)- 0.5*get_calibrated_prox(IR2);
	rightSpeed =0.39*MOTOR_SPEED_LIMIT;
	if (end_right_wall()&&(!debut)){
		debut=left_motor_get_pos();
		set_led(LED5,1);
	}
	if(debut && (!fin) && (get_calibrated_prox(IR3)>450)){
		fin=left_motor_get_pos();
		empty_space_dimension=fin-debut;
		debut=0,fin=0;
		set_led(LED5,0);
	}
	if(empty_space_dimension>=PLACE_DIM_MIN){
		to_computer_dim=empty_space_dimension;
		empty_space_dimension=-1;
		return true;
	}else{
		empty_space_dimension=-1;
		return false;
	}
}

void sortie_park(void){
	static bool tourne=false;

	clignotant(LED8,LED6);

	if(!tourne){
		leftSpeed= 0.5*MOTOR_SPEED_LIMIT-0.25*get_calibrated_prox(IR2);
		rightSpeed= 0.5*MOTOR_SPEED_LIMIT-0.25*get_calibrated_prox(IR4);
		if ( (get_calibrated_prox(IR3)<30)&&(get_calibrated_prox(IR6)<30)&&(get_calibrated_prox(IR4)<30)&&(get_calibrated_prox(IR5)<30)){
				tourne=true;
				left_motor_set_pos(0);
				right_motor_set_pos(0);
			}
	}else if(tourne){
		leftSpeed=-0.5*MOTOR_SPEED_LIMIT;
		rightSpeed=0.5*MOTOR_SPEED_LIMIT;
		if(right_motor_get_pos()>=(CORRECTION_ROTATION_90*POSITION_ROTATION_90)){
			tourne=false;
			done=true;
		}
	}

}

void park(void){
	static bool place_found=0, tourne_marche_arriere=true;

	if (place_found){

		if(tourne_marche_arriere){
			leftSpeed=-0.5*MOTOR_SPEED_LIMIT;
			rightSpeed =-0.13*MOTOR_SPEED_LIMIT;
			if (get_calibrated_prox(IR4)>150){
				tourne_marche_arriere=false;
				}
		}else if(!tourne_marche_arriere){
			leftSpeed=-0.5*MOTOR_SPEED_LIMIT+5*get_calibrated_prox(IR4);
			rightSpeed =-0.5*MOTOR_SPEED_LIMIT+0.25*get_calibrated_prox(IR2);
			if( (get_calibrated_prox(IR4)<30)&&(get_calibrated_prox(IR5)<30)&&(get_calibrated_prox(IR2)>200) ){
				parkdone=true;
				leftSpeed=0;
				rightSpeed=0;
				tourne_marche_arriere=true;
				place_found=false;
				}
			}
	}else{
		place_found=find_a_place();
	}
}





void send_to_computer(TO_DO instruction){
	static uint8_t mustSend = 0;
	if(mustSend > 8){
	chprintf((BaseSequentialStream *)&SD3, "sound= %f \r\n",get_norm());
	chprintf((BaseSequentialStream *)&SD3, "place dimension= %d \r\n",to_computer_dim);
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
    TO_DO instruction=0;
    TO_DO instruction_tab[MAX_NB_INSTRUCTION]={0};
    int8_t nb_instruction=-1;
    bool wayback=0;
    done=true;
    parkdone=false;


    //waits until a start signal (FREQ_START sound) is detected
	wait_start_signal();
	calibrate_ir();

    while(1){
		time = chVTGetSystemTime();

		if(!wayback){
			if (done) {
				clear_leds();
				if ( obstacle_detected(OBS_DIST_38cm,THRESHOLD_38cm) && (get_next_instruction()== PARK) ){
						instruction=PARK;
						done=false;
				}else if(obstacle_detected(OBS_DIST_15cm,THRESHOLD_15cm)){
					nb_instruction++;
					instruction=get_next_instruction();
					instruction_tab[nb_instruction]=instruction;
					done=false;
				}else{
					leftSpeed=DEFAULT_SPEED-0.8*get_calibrated_prox(IR1) - 0.4*get_calibrated_prox(IR2);
					rightSpeed=DEFAULT_SPEED-0.8*get_calibrated_prox(IR8) - 0.4*get_calibrated_prox(IR7);
				}
					}
		}else if(wayback){
			playMelody(IMPOSSIBLE_MISSION, ML_SIMPLE_PLAY, NULL);
			if (done) {
				clear_leds();
				if(obstacle_detected(OBS_DIST_15cm,THRESHOLD_15cm)){
					instruction=wayback_instruction(instruction_tab[nb_instruction]);
					nb_instruction--;
					done=false;
				}else if(nb_instruction == -1){
					//THE END
					stopCurrentMelody();
					leftSpeed=0;
					rightSpeed=0;
				}else{
					leftSpeed=DEFAULT_SPEED-0.8*get_calibrated_prox(IR1) - 0.4*get_calibrated_prox(IR2);
					rightSpeed=DEFAULT_SPEED-0.8*get_calibrated_prox(IR8) - 0.4*get_calibrated_prox(IR7);				}
			}
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
				case RONDPOINT_EXIT1...RONDPOINT_EXIT4 :
					rond_point(instruction);
					break;
				case PARK:
					if(!wayback){
						park();
					}else{
						sortie_park();
					}
					break;
				default:
					instruction=ERREUR;
					leftSpeed=0;
					rightSpeed=0;
					break;
			}
		}

		send_to_computer(instruction);

		left_motor_set_speed(leftSpeed);
		right_motor_set_speed(rightSpeed);

		if (parkdone){
			wayback=true;
			parkdone=false;
			clear_leds();
			chThdSleepMilliseconds(5000);
		}

		chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz
	}
 }

void movement_start(void){
	chThdCreateStatic(waMovement, sizeof(waMovement), NORMALPRIO, Movement, NULL);
}
