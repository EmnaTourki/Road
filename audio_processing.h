#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#include <move.h>

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT
} BUFFER_NAME_t;

void processAudioData(int16_t *data, uint16_t num_samples);

void wait_start_signal(void);

TO_DO get_next_instruction(void);

TO_DO wayback_instruction(TO_DO);

float get_norm(void);

#endif /* AUDIO_PROCESSING_H */
