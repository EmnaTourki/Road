
#ifndef MOVE_H_
#define MOVE_H_

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
