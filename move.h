
#ifndef MOVE_H_
#define MOVE_H_


/*
 * List of the different instructions that can have the robot
 *
 * START,TURN_RIGHT,TURN_LEFT,RONDPOINT_EXIT[1,2,3,4],PARK are detected by audio processing with the microphone
 * PASSAGE_PIETON is detected by image processing with the camera
 */
typedef enum{
	START=-2,
	TURN_RIGHT,
	TURN_LEFT,
	RONDPOINT_EXIT1,
	RONDPOINT_EXIT2,
	RONDPOINT_EXIT3,
	RONDPOINT_EXIT4,
	PARK,
	PASSAGE_PIETON,
	ERREUR
} TO_DO;


//List of the different IR sensors
typedef enum{
	IR1=0,
	IR2,
	IR3,
	IR4,
	IR5,
	IR6,
	IR7,
	IR8
}IR;


//start the movement thread
void movement_start(void);

#endif /* MOVE_H_ */
