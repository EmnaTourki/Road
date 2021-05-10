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
#include <process_image.h>
#include <audio_processing.h>
#include <audio/play_melody.h>
#include <audio/audio_thread.h>
#include <camera/po8030.h>



#define PI                  		3.1415926536f
#define WHEEL_DISTANCE      		5.30f   					//cm
#define PERIMETER_EPUCK     		(PI * WHEEL_DISTANCE)		//cm
#define NSTEP_ONE_TURN      		1000 						// number of step for 1 turn of the motor
#define WHEEL_PERIMETER				13 							// cm
#define POSITION_ROTATION_90 		(0.25*PERIMETER_EPUCK) * NSTEP_ONE_TURN / WHEEL_PERIMETER 	//step
#define CORRECTION_ROTATION_90 		1
#define DEFAULT_SIDE_LENGTH			1000						// steps
#define DEFAULT_SPEED				0.6 * MOTOR_SPEED_LIMIT 	// step/s
#define LOW_SPEED					0.5 * MOTOR_SPEED_LIMIT 	// step/s
#define OBS_DIST_15cm				130 						// mm
#define OBS_DIST_38cm				380							// mm
#define THRESHOLD_15cm				20							// mm
#define THRESHOLD_38cm				80							// mm
#define PLACE_DIM_MIN 				480							// steps
#define MAX_NB_INSTRUCTION			20
#define NO_INSTRUCTION				-1
#define CLOSE_OBST_IR_VALUE			500
#define OBST_PASSED_IR_VALUE		200
#define NO_OBST_IR_VALUE			30
#define LED_INTENSITY				255

static int16_t leftSpeed = 0, rightSpeed = 0;
static bool done=true;
static bool parkdone=false;
static int32_t to_computer_dim=0;


/*
 * This function uses the TOF sensor to calculate the distance from the obstacle.
 * Returns true if an obstacle is around obst_dist mm away depending on the threshold taken.
 */
bool obstacle_detected(uint16_t obst_dist,uint8_t threshold ){
	return (VL53L0X_get_dist_mm()>=(obst_dist-threshold))
			&& (VL53L0X_get_dist_mm()<=(obst_dist+threshold));
}

/*
 * Detects the wall/obstacle on the left of the e_puck using the IR6 sensor
 * Returns true if that wall/obstacle comes to an end.
 * Needs to be called in a loop.
 */
bool end_left_wall(void){
	static bool walldetected=0;
	if (get_calibrated_prox(IR6)>CLOSE_OBST_IR_VALUE){
		walldetected=true;
	}
	if((walldetected)&&(get_calibrated_prox(IR6)<OBST_PASSED_IR_VALUE)){
		walldetected=false;
		return true;
	}else {return false;}
}

/*
 * Detects the wall/obstacle on the right of the e_puck using the IR3 sensor
 * Returns true if that wall/obstacle comes to an end.
 * Needs to be called in a loop.
 */
bool end_right_wall(void){
	static bool walldetected=0;
	if (get_calibrated_prox(IR3)>CLOSE_OBST_IR_VALUE){
		walldetected=true;
	}
	if	((walldetected)&&(get_calibrated_prox(IR3)<OBST_PASSED_IR_VALUE)){
		walldetected=false;
		return true;
	}else {return false;}
}

/*
 * A function to simulate the turn signals of a car.
 * Needs to be called in a loop.
 *
 * @param :led_1 and led_2 are the LEDs that will blink.
 */
void clignotant(rgb_led_name_t led_1,rgb_led_name_t led_2){
	static uint8_t counter=0;
	if(counter>10){
		//GREEN+RED=YELLOW LIGHT
		toggle_rgb_led(led_1, GREEN_LED, LED_INTENSITY);
		toggle_rgb_led(led_1, RED_LED, LED_INTENSITY);

		toggle_rgb_led(led_2, GREEN_LED, LED_INTENSITY);
		toggle_rgb_led(led_2, RED_LED, LED_INTENSITY);

		counter=0;
	}
	counter++;
}

/*
 * Rotate the e_puck right at the next obstacle using the IR7 and IR8 front left sensors.
 * Called in a loop until done becomes true.
 */
void rotate_right(void){
	//Right Turn Signal
	clignotant(LED2,LED4);

	leftSpeed=0.5*MOTOR_SPEED_LIMIT;
	rightSpeed=0.5*MOTOR_SPEED_LIMIT- 1.4*get_calibrated_prox(IR8) - 0.42*get_calibrated_prox(IR7);

	if (end_left_wall()){
	//Breaks out of the loop when the end of the obstacle is detected.
		done=true;
	}
}

/*
 * Rotate the e_puck left at the next obstacle using the IR1 and IR2 front right sensors.
 * Called in a loop until done becomes true.
 */
void rotate_left(void){
	//Left Turn Signal
	clignotant(LED8,LED6);

	leftSpeed=0.5*MOTOR_SPEED_LIMIT- 1.4*get_calibrated_prox(IR1) - 0.42*get_calibrated_prox(IR2);
	rightSpeed =0.5*MOTOR_SPEED_LIMIT;

	if (end_right_wall()){
	//Breaks out of the loop when the end of the obstacle is detected.
		done=true;
	}
}

/*
 * Used to exit the "roundabout"
 * Called in a loop until done becomes true.
 *
 * @param :int32_t position_to_reach is the position in steps where we want to exit the "roundabout"
 */
void sortie_rondpoint(int32_t position_to_reach){
	static bool reset_position=0;

	//Right Turn Signal
	clignotant(LED2,LED4);

	if(!reset_position){
		//first we reset the step counter
		left_motor_set_pos(0);
		right_motor_set_pos(0);
		reset_position=true;
	}else if((reset_position)&&(left_motor_get_pos()>(position_to_reach))){
		//when we reach the middle of the roundabout side (value of position_to_reach) , we start a clockwise rotation of 90 degrees
		leftSpeed= 0.5*MOTOR_SPEED_LIMIT;
		rightSpeed =-0.5* MOTOR_SPEED_LIMIT;

		if((left_motor_get_pos()>=(position_to_reach+CORRECTION_ROTATION_90*POSITION_ROTATION_90))){
			//once the rotation is complete,we break out of the loop by setting done true
			done=true;
			reset_position=false;
		}
	}
}

/*
 * In our route, the roundabout is square.
 *
 */
void rond_point(uint8_t exit_nbr){
	static uint8_t nb_turn=0;
	static bool turningleft=0;
	static int32_t debut=0 ,fin=0,position_to_reach=0.5*DEFAULT_SIDE_LENGTH;
	if(nb_turn==0){
		leftSpeed= 0.45*MOTOR_SPEED_LIMIT;
		rightSpeed =0.5*MOTOR_SPEED_LIMIT-0.7*get_calibrated_prox(IR8) - 0.25*get_calibrated_prox(IR7);
	}
	if( (!turningleft)){
		if( end_left_wall() && (nb_turn!=exit_nbr)){
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
			if((nb_turn==exit_nbr)){
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
		if(get_calibrated_prox(IR7)>CLOSE_OBST_IR_VALUE){
			turningleft=false;
			nb_turn++;
		}
	}
}

/*
 *
 */
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

/*
 *
 */
void sortie_park(void){
	static bool tourne=false;

	clignotant(LED8,LED6);

	if(!tourne){
		leftSpeed= 0.5*MOTOR_SPEED_LIMIT-0.25*get_calibrated_prox(IR2);
		rightSpeed= 0.5*MOTOR_SPEED_LIMIT-0.25*get_calibrated_prox(IR4);
		if ( (get_calibrated_prox(IR3)<NO_OBST_IR_VALUE)&&(get_calibrated_prox(IR6)<NO_OBST_IR_VALUE)
				&&(get_calibrated_prox(IR4)<NO_OBST_IR_VALUE)&&(get_calibrated_prox(IR5)<NO_OBST_IR_VALUE)){
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

/*
 *
 */
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
			if( (get_calibrated_prox(IR4)<NO_OBST_IR_VALUE)&&(get_calibrated_prox(IR5)<NO_OBST_IR_VALUE)&&(get_calibrated_prox(IR2)>OBST_PASSED_IR_VALUE) ){
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

void stop_motors(void){
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

/*
 *
 */
void pedestrian_crossing(void){
	stop_motors();
	chThdSleepMilliseconds(3000);
	left_motor_set_speed(DEFAULT_SPEED);
	right_motor_set_speed(DEFAULT_SPEED);
	chThdSleepMilliseconds(2000);
	done=true;
}

/*
 *
 */
void send_to_computer(TO_DO instruction){
	static uint8_t mustSend = 0;
	if(mustSend > 8){
	chprintf((BaseSequentialStream *)&SD3, "lineWidth= %d \r\n", get_lineWidth());
	chprintf((BaseSequentialStream *)&SD3, "sound= %f \r\n",get_norm());
	chprintf((BaseSequentialStream *)&SD3, "place dimension= %d \r\n",to_computer_dim);
	chprintf((BaseSequentialStream *)&SD3, "INSTRUCTION= %d,distance= %d \r\n",instruction,VL53L0X_get_dist_mm());
	chprintf((BaseSequentialStream *)&SD3, "speed: %4d,%4d,\r\n",leftSpeed,rightSpeed);
	chprintf((BaseSequentialStream *)&SD3, "Calibrated IR: %4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,\r\n", get_calibrated_prox(IR1), get_calibrated_prox(IR2),
	get_calibrated_prox(IR3),get_calibrated_prox(IR4), get_calibrated_prox(IR5), get_calibrated_prox(IR6), get_calibrated_prox(IR7), get_calibrated_prox(IR8));
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
    int8_t nb_instruction=NO_INSTRUCTION;
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
				if (passage_pieton()){
					instruction=PASSAGE_PIETON;
					done=false;
				}
				else if( obstacle_detected(OBS_DIST_38cm,THRESHOLD_38cm) && (get_next_instruction()== PARK) ){
						instruction=PARK;
						done=false;
				}
				else if(obstacle_detected(OBS_DIST_15cm,THRESHOLD_15cm)){
					nb_instruction++;
					instruction=get_next_instruction();
					instruction_tab[nb_instruction]=instruction;
					done=false;
				}
				else{
					leftSpeed=DEFAULT_SPEED-0.8*get_calibrated_prox(IR1) - 0.4*get_calibrated_prox(IR2);
					rightSpeed=DEFAULT_SPEED-0.8*get_calibrated_prox(IR8) - 0.4*get_calibrated_prox(IR7);
				}
					}
		}else if(wayback){
			playMelody(IMPOSSIBLE_MISSION, ML_SIMPLE_PLAY, NULL);
			if (done) {
				clear_leds();
				if (passage_pieton()){
					instruction=PASSAGE_PIETON;
					done=false;
				}
				else if(obstacle_detected(OBS_DIST_15cm,THRESHOLD_15cm)){
					instruction=wayback_instruction(instruction_tab[nb_instruction]);
					nb_instruction--;
					done=false;
				}
				else if(nb_instruction == NO_INSTRUCTION){
					//THE END
					stopCurrentMelody();
					leftSpeed=0;
					rightSpeed=0;
				}
				else{
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
				case PASSAGE_PIETON:
					pedestrian_crossing();
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
