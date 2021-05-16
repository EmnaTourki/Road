#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <math.h>

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




#define PI                  			3.1415926536f
#define WHEEL_DISTANCE      			5.30f   					//cm
#define PERIMETER_EPUCK     			(PI * WHEEL_DISTANCE)		//cm
#define NSTEP_ONE_TURN      			1000 						// number of step for 1 turn of the motor
#define WHEEL_PERIMETER					13 							// cm

#define POSITION_ROTATION_90 			(0.25*PERIMETER_EPUCK) * NSTEP_ONE_TURN / WHEEL_PERIMETER 	//step
#define CORRECTION_ROTATION_90 			1
#define DEFAULT_SIDE_LENGTH				1000						// steps
#define DEFAULT_SPEED					0.6 * MOTOR_SPEED_LIMIT 	// step/s
#define LOW_SPEED_COEF					0.105
#define LOW_SPEED_COEF_PARK				0.13
#define NORMAL_SPEED_COEF				0.5
#define LOWER_SPEED_COEF				0.45
#define LOWER_SPEED_COEF_PARK			0.39
#define OBS_DIST_15cm					130 						// mm
#define OBS_DIST_38cm					380							// mm
#define THRESHOLD_15cm					20							// mm
#define THRESHOLD_38cm					80							// mm
#define PLACE_DIM_MIN 					480							// steps
#define MAX_NB_INSTRUCTION				50
#define NO_INSTRUCTION					-1
#define CLOSE_OBST_IR_VALUE				500
#define DETECT_OBST_IR_VALUE			450
#define OBST_PASSED_IR_VALUE			200
#define OBST_DETECT_BACK_IR_VALUE		150
#define NO_OBST_IR_VALUE				30
#define ROTATE_H_IR_COEF				1.4
#define ROTATE_L_IR_COEF				0.42
#define ROTATION_IR_COEF				0.5
#define ROTATION_H_IR_COEF				0.7
#define ROTATION_L_IR_COEF				0.25
#define ROTATION_H_COEF					5
#define AVOID_OBST_H_COEF				0.8
#define AVOID_OBST_L_COEF				0.4
#define PARK_WAIT_TIME					5000
#define PEDESTRIAN_CROSSING_WAIT_TIME	3000
#define FREQUENCY_100HZ					10
#define LED_INTENSITY					255
#define MAX_COUNTER						8
#define MAX_COUNTER_CLIGNOTANT			10
#define FIRST_TURN						1
#define SECOND_TURN						2


static int16_t leftSpeed = 0, rightSpeed = 0;
static uint8_t done=true;
static uint8_t parkdone=false;
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
	static uint8_t walldetected=false;
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
	static uint8_t walldetected=false;
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
	if(counter>MAX_COUNTER_CLIGNOTANT){
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
 * Rotates the e_puck right at the next obstacle using the IR7 and IR8 front left sensors.
 * Called in a loop until done becomes true.
 */
void rotate_right(void){
	//Right Turn Signal
	clignotant(LED2,LED4);

	leftSpeed=NORMAL_SPEED_COEF*MOTOR_SPEED_LIMIT;
	rightSpeed=NORMAL_SPEED_COEF*MOTOR_SPEED_LIMIT- ROTATE_H_IR_COEF*get_calibrated_prox(IR8)- ROTATE_L_IR_COEF*get_calibrated_prox(IR7);

	if (end_left_wall()){
	//Breaks out of the loop when the end of the obstacle is detected.
		done=true;
	}
}

/*
 * Rotates the e_puck left at the next obstacle using the IR1 and IR2 front right sensors.
 * Called in a loop until done becomes true.
 */
void rotate_left(void){
	//Left Turn Signal
	clignotant(LED8,LED6);

	leftSpeed=NORMAL_SPEED_COEF*MOTOR_SPEED_LIMIT- ROTATE_H_IR_COEF*get_calibrated_prox(IR1) - ROTATE_L_IR_COEF*get_calibrated_prox(IR2);
	rightSpeed =NORMAL_SPEED_COEF*MOTOR_SPEED_LIMIT;

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
	static uint8_t reset_position=false;

	//Right Turn Signal
	clignotant(LED2,LED4);

	if(!reset_position){
		//first we reset the step counter
		left_motor_set_pos(0);
		right_motor_set_pos(0);
		reset_position=true;
	}else if((reset_position)&&(left_motor_get_pos()>(position_to_reach))){
		//when we reach the middle of the roundabout side (value of position_to_reach) , we start a clockwise rotation of 90 degrees
		leftSpeed= NORMAL_SPEED_COEF*MOTOR_SPEED_LIMIT;
		rightSpeed =-NORMAL_SPEED_COEF* MOTOR_SPEED_LIMIT;

		if((left_motor_get_pos()>=(position_to_reach+CORRECTION_ROTATION_90*POSITION_ROTATION_90))){
			//once the rotation is complete,we break out of the loop by setting done true
			done=true;
			reset_position=false;
		}
	}
}

/*
 * This function is called in a loop until done becomes true.
 * In our route, the roundabout is square shaped.There is then 4 exit possibilities (1,2,3 or 4)
 * Depending on the parameter exit_nbr entered , the e_puck will follow the wall and turn left exit_nbr times
 * then the function sortie_rondpoint is called to exit.
 *
 * @param :uint8_t exit_nbr is the number of left turns the e_puck make before exiting the roundabout.
 */
void rond_point(uint8_t exit_nbr){
	static uint8_t nb_turn=0,turningleft=false;
	static int32_t debut=0 ,fin=0;
	static int32_t position_to_reach=0.5*DEFAULT_SIDE_LENGTH;
	if(nb_turn==0){
		//we rotate right when entering the roundabout using the IR7 and IR8 front left sensors.
		leftSpeed= LOWER_SPEED_COEF*MOTOR_SPEED_LIMIT;
		rightSpeed =NORMAL_SPEED_COEF*MOTOR_SPEED_LIMIT-ROTATION_H_IR_COEF*get_calibrated_prox(IR8) - ROTATION_L_IR_COEF*get_calibrated_prox(IR7);
	}
	if( (!turningleft)){
		//we're following the wall
		if( end_left_wall() && (nb_turn!=exit_nbr)){
			//we start turning left when the wall comes to an end.
			turningleft=true;
		}

		if (nb_turn>=FIRST_TURN){
			//we follow the wall which is always in our left by leaning a little to the left
			//and having the IR left sensors swerve us to the right if too close.
			leftSpeed= LOWER_SPEED_COEF*MOTOR_SPEED_LIMIT;
			rightSpeed =NORMAL_SPEED_COEF*MOTOR_SPEED_LIMIT-ROTATION_IR_COEF*get_calibrated_prox(IR8)- ROTATION_L_IR_COEF*get_calibrated_prox(IR7);

			//here we want to know where is the middle of one side of our square roundabout
			//we calculate it between the first and second turn
			if((nb_turn==FIRST_TURN)&&(!debut)){
				debut=left_motor_get_pos();
			}
			if((nb_turn==SECOND_TURN)&&(debut)&&(!fin)){
				fin=left_motor_get_pos();
				position_to_reach=0.5*(fin-debut);
			}
			if((nb_turn==exit_nbr)){
				//we just turned the last turn and we want to exit the roundabout
				//we put that length calculated as parameter to the function sortie_rondpoint
				sortie_rondpoint(position_to_reach);

				//once we're out,done is set true in the function sortie_rondpoint
				if (done){
					//reinitialize variable
					nb_turn=0,debut=0,fin=0,turningleft=false;
				}
			}

		}
	}
	else if(turningleft){
		//we're turning
		leftSpeed=LOW_SPEED_COEF*MOTOR_SPEED_LIMIT;
		rightSpeed =NORMAL_SPEED_COEF*MOTOR_SPEED_LIMIT;
		if(get_calibrated_prox(IR7)>CLOSE_OBST_IR_VALUE){
			//we stop turning once we detect the wall with IR7 sensor
			turningleft=false;
			//increment the number of turns
			nb_turn++;
		}
	}
}

/*
 * Used to find a place to park.
 * Returns true if a place > PLACE_DIM_MIN is found.
 * Called in a loop.
 */
bool find_a_place(void){
	static int32_t debut=0 , fin=0, empty_space_dimension=-1;
	//we search for a place at our right
	//we lean to the right and let the IR right sensors swerve us to the left if too close to an obstacle.
	leftSpeed=NORMAL_SPEED_COEF*MOTOR_SPEED_LIMIT-ROTATION_IR_COEF*get_calibrated_prox(IR1)- ROTATION_IR_COEF*get_calibrated_prox(IR2);
	rightSpeed =LOWER_SPEED_COEF_PARK*MOTOR_SPEED_LIMIT;

	if (end_right_wall()&&(!debut)){
		//start of an empty place detected
		//we begin counting steps
		debut=left_motor_get_pos();
		//back led set on
		set_led(LED5,1);
	}
	if(debut && (!fin) && (get_calibrated_prox(IR3)>DETECT_OBST_IR_VALUE)){
		//end of the empty place detected
		//we calculate its dimension
		fin=left_motor_get_pos();
		empty_space_dimension=fin-debut;
		//reinitialize variable
		debut=0,fin=0;
		//back led turned off
		set_led(LED5,0);
	}
	if(empty_space_dimension>=PLACE_DIM_MIN){
		//if the place is large enough for the e_puck to enter it, return true
		to_computer_dim=empty_space_dimension;
		//reinitialize variable
		empty_space_dimension=-1;
		return true;
	}else{
		empty_space_dimension=-1;
		return false;
	}
}

/*
 * Called to exit the parking place.The e_puck goes forward then turns left.
 * Called in a loop until done becomes true.
 */
void sortie_park(void){
	static uint8_t tourne=false;
	////Left Turn Signal
	clignotant(LED8,LED6);

	if(!tourne){
		//The e_puck moves forward without approaching the obstacles thanks to the IR sensors
		leftSpeed= NORMAL_SPEED_COEF*MOTOR_SPEED_LIMIT-ROTATION_L_IR_COEF*get_calibrated_prox(IR2);
		rightSpeed= NORMAL_SPEED_COEF*MOTOR_SPEED_LIMIT-ROTATION_L_IR_COEF*get_calibrated_prox(IR4);

		if ( (get_calibrated_prox(IR3)<NO_OBST_IR_VALUE)&&(get_calibrated_prox(IR6)<NO_OBST_IR_VALUE)
				&&(get_calibrated_prox(IR4)<NO_OBST_IR_VALUE)&&(get_calibrated_prox(IR5)<NO_OBST_IR_VALUE)){
			//once out of the parking space, starts a counterclockwise rotation of 90 degrees
				tourne=true;
			//we reset the step counter
				left_motor_set_pos(0);
				right_motor_set_pos(0);
			}
	}else if(tourne){
		//turning
		leftSpeed= -NORMAL_SPEED_COEF*MOTOR_SPEED_LIMIT;
		rightSpeed= NORMAL_SPEED_COEF*MOTOR_SPEED_LIMIT;
		if(right_motor_get_pos()>=(CORRECTION_ROTATION_90*POSITION_ROTATION_90)){
			//once the desired number of step reached ,the rotation is complete, we break out of the loop by setting done true
			done=true;
			//reinitialize variable
			tourne=false;
		}
	}
}

/*
 * Used to park the e_puck in reverse between two obstacles
 * Called in a loop until parkdone becomes true.
 */
void park(void){
	static uint8_t place_found=false, tourne_marche_arriere=true;

	if (place_found){
		//enters the parking space in reverse
		if(tourne_marche_arriere){
			//rotates right in reverse
			leftSpeed=-NORMAL_SPEED_COEF*MOTOR_SPEED_LIMIT;
			rightSpeed =-LOW_SPEED_COEF_PARK*MOTOR_SPEED_LIMIT;
			if (get_calibrated_prox(IR4)>OBST_DETECT_BACK_IR_VALUE){
				//stop rotating when obstacle detected with the back right sensor IR4
				tourne_marche_arriere=false;
				}
		}else if(!tourne_marche_arriere){
			//While continuing to move backward,turns using the sensors to position itself parallel to the obstacle.
			leftSpeed= -NORMAL_SPEED_COEF*MOTOR_SPEED_LIMIT+ROTATION_H_COEF*get_calibrated_prox(IR4);
			rightSpeed = -NORMAL_SPEED_COEF*MOTOR_SPEED_LIMIT+ROTATION_L_IR_COEF*get_calibrated_prox(IR2);

			if( (get_calibrated_prox(IR4)<NO_OBST_IR_VALUE)&&(get_calibrated_prox(IR5)<NO_OBST_IR_VALUE)&&(get_calibrated_prox(IR2)>OBST_PASSED_IR_VALUE) ){
				//park accomplished,stops the motors
				leftSpeed=0;
				rightSpeed=0;
				parkdone=true;
				//reinitialize variables
				tourne_marche_arriere=true;
				place_found=false;
				}
			}
	}else{
		//if place not found, continue the search .
		place_found=find_a_place();
	}
}

/*
 * stops the motors
 */
 void stop_motors(void){
	 left_motor_set_speed(0);
	 right_motor_set_speed(0);
 }

/*
 * Used to cross the pedestrian crossing
 * stop if obstacle detected
 * Called in a loop until done becomes true.
 */
void pedestrian_crossing(void){
	static uint8_t pedestrian_detected=false;
	if((get_calibrated_prox(IR1)<=DETECT_OBST_IR_VALUE)&&(get_calibrated_prox(IR8)<=DETECT_OBST_IR_VALUE)){
		//no pedestrian\obstacle detected
		pedestrian_detected=false;
		leftSpeed=NORMAL_SPEED_COEF*MOTOR_SPEED_LIMIT;
		rightSpeed=NORMAL_SPEED_COEF*MOTOR_SPEED_LIMIT;
	}else{
		//obstacle detected
		pedestrian_detected=true;
		//stops
		leftSpeed=0;
		rightSpeed=0;
	}
	if (fin_passage_pieton()&&(!pedestrian_detected)){
	//crossed
		done=true;
	}
}
/*
 * Used to send informations to computer
 * @param TO_DO instruction : the instruction we're currently doing
 */
void send_to_computer(TO_DO instruction){
	static uint8_t mustSend = 0;
	if(mustSend > MAX_COUNTER){
	chprintf((BaseSequentialStream *)&SD3, "lineWidth= %d \r\n", get_lineWidth());
	chprintf((BaseSequentialStream *)&SD3, "sound= %f \r\n",get_norm());
	chprintf((BaseSequentialStream *)&SD3, "place dimension= %d \r\n",to_computer_dim);
	chprintf((BaseSequentialStream *)&SD3, "INSTRUCTION= %d,distance= %d \r\n",instruction,VL53L0X_get_dist_mm());
	chprintf((BaseSequentialStream *)&SD3, "speed: %4d,%4d,\r\n",leftSpeed,rightSpeed);
	chprintf((BaseSequentialStream *)&SD3, "Calibrated IR: %4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,\r\n\n", get_calibrated_prox(IR1), get_calibrated_prox(IR2),
	get_calibrated_prox(IR3),get_calibrated_prox(IR4), get_calibrated_prox(IR5), get_calibrated_prox(IR6), get_calibrated_prox(IR7), get_calibrated_prox(IR8));
	mustSend = 0;
	}
	mustSend++;
}

static THD_WORKING_AREA(waMovement, 2048);
static THD_FUNCTION(Movement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    TO_DO instruction=0;
    TO_DO instruction_tab[MAX_NB_INSTRUCTION]={0};
    int8_t nb_instruction=NO_INSTRUCTION;
    uint8_t wait=false,wayback=false,the_end=false;
    done=true;
    parkdone=false;


    //waits until a start signal (FREQ_START sound) is detected
	wait_start_signal();
	//calibrate IR sensors
	calibrate_ir();

    while(1){
		time = chVTGetSystemTime();

		if(!wayback){
			//on the way out
			if (done) {
				// if the previous instruction is done, we keep moving forward until we take a new instruction
				clear_leds();

				if (passage_pieton()){
					// a pedestrian crossing is detected with the camera
					instruction=PASSAGE_PIETON;
					wait=true;
					done=false;
				}
				else if( obstacle_detected(OBS_DIST_38cm,THRESHOLD_38cm) && (get_next_instruction()== PARK) ){
					// an obstacle(representing a road sign) is detected around 38 cm away and the last instruction heard is to park
						instruction=PARK;
						done=false;
				}
				else if(obstacle_detected(OBS_DIST_15cm,THRESHOLD_15cm)){
					//an obstacle(representing a road sign) is detected around 15 cm away
					//check what's the instruction to do
					instruction=get_next_instruction();
					nb_instruction++;
					//record the instructions in a tab so that we can do the way back based on it.
					instruction_tab[nb_instruction]=instruction;
					done=false;
				}
				else{
					//if no instruction was taken, we move forward and deviate to the right or left if any obstacle (wall) is too close.
					leftSpeed=DEFAULT_SPEED-AVOID_OBST_H_COEF*get_calibrated_prox(IR1) - AVOID_OBST_L_COEF*get_calibrated_prox(IR2);
					rightSpeed=DEFAULT_SPEED-AVOID_OBST_H_COEF*get_calibrated_prox(IR8) - AVOID_OBST_L_COEF*get_calibrated_prox(IR7);
				}
			}
		}
		else if((wayback)&&(!the_end)){
			//on the way back home
			//play a melody
			playMelody(IMPOSSIBLE_MISSION, ML_SIMPLE_PLAY, NULL);
			if (done) {
				// if the previous instruction is done, we keep moving forward until we take a new instruction
				clear_leds();
				if (passage_pieton()){
					// a pedestrian crossing is detected with the camera
					instruction=PASSAGE_PIETON;
					wait=true;
					done=false;
				}else if(nb_instruction == NO_INSTRUCTION){
					//if we have already done the last instruction in the table
					//THE END
					the_end=true;
					//stops the melody and stops the motors
					stopCurrentMelody();
					leftSpeed=0;
					rightSpeed=0;
				}else if(obstacle_detected(OBS_DIST_15cm,THRESHOLD_15cm)){
					//an obstacle(representing a road sign) is detected around 15 cm away
					//check what's the instruction to do based on the table filled in on the way out
					instruction=wayback_instruction(instruction_tab[nb_instruction]);
					nb_instruction--;
					done=false;
				}else{
					//if no instruction was taken, we move forward and deviate to the right or left if any obstacle (wall) is too close.
					leftSpeed=DEFAULT_SPEED-AVOID_OBST_H_COEF*get_calibrated_prox(IR1) - AVOID_OBST_L_COEF*get_calibrated_prox(IR2);
					rightSpeed=DEFAULT_SPEED-AVOID_OBST_H_COEF*get_calibrated_prox(IR8) - AVOID_OBST_L_COEF*get_calibrated_prox(IR7);
				}
			}
		}


		if (!done){
			//do the given instruction until done becomes true
			switch (instruction)
			{
				case TURN_RIGHT:
					rotate_right();
					break;
				case TURN_LEFT:
					rotate_left();
					break;
				case RONDPOINT_EXIT1...RONDPOINT_EXIT4 :
				//this function takes as parameter the number of left turns the e_puck makes before exiting the roundabout.
				//which corresponds to the enumeration of the instruction taken
					rond_point(instruction);
					break;
				case PARK:
					if(!wayback){
						park();
					}else{
						//because park is always the last instruction taken on the way out
						//this is always the first instruction to do on the way back home
						sortie_park();
					}
					break;
				case PASSAGE_PIETON:
					if(wait){
						//stop the motors and wait 3 seconds
						stop_motors();
						chThdSleepMilliseconds(PEDESTRIAN_CROSSING_WAIT_TIME);
						wait=false;
					}
					//cross the pedestrian crossing if no obstacle(pedestrian) is detected
					pedestrian_crossing();
					break;
				default:
					leftSpeed=0;
					rightSpeed=0;
					break;
			}
		}

		send_to_computer(instruction);

		left_motor_set_speed(leftSpeed);
		right_motor_set_speed(rightSpeed);

		if (parkdone){
			//reinitialize variable
			parkdone=false;
			clear_leds();
			//park is always the last instruction in every set of instructions on the way out
			//once the park done , wait 5 seconds
			chThdSleepMilliseconds(PARK_WAIT_TIME);
			//then start the way back home by leaving the parking so without changing the instruction "PARK"
			wayback=true;
		}

		chThdSleepUntilWindowed(time, time + MS2ST(FREQUENCY_100HZ)); // Refresh @ 100 Hz
	}
 }

void movement_start(void){
	chThdCreateStatic(waMovement, sizeof(waMovement), NORMALPRIO, Movement, NULL);
}
