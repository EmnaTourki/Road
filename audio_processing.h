#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT
} BUFFER_NAME_t;

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


void processAudioData(int16_t *data, uint16_t num_samples);

void wait_start_signal(void);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

/*
*	Returns what to do at the next obstacle
*/
TO_DO get_next_instruction(void);


TO_DO wayback_instruction(TO_DO);

float get_norm(void);

#endif /* AUDIO_PROCESSING_H */
