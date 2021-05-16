//amended from tp5_Noisy

#include "ch.h"
#include "hal.h"

#include <usbcfg.h>
#include <chprintf.h>

#include <move.h>
#include <audio/microphone.h>
#include <audio_processing.h>

#include <arm_math.h>
#include <arm_const_structs.h>

#define FFT_SIZE 	1024
#define MIN_VALUE_THRESHOLD		50000.0f

#define MIN_FREQ				24	//375Hz we don't analyze before this index to not use resources for nothing
#define FREQ_START				26	//406Hz
#define FREQ_RIGHT				32	//500Hz
#define FREQ_LEFT				35	//546Hz
#define FREQ_RONDPOINT_EXIT1	38	//593HZ
#define FREQ_RONDPOINT_EXIT2	41	//640Hz
#define FREQ_RONDPOINT_EXIT3	44	//687Hz
#define FREQ_RONDPOINT_EXIT4	47	//734Hz
#define FREQ_PARK				50	//781Hz
#define MAX_FREQ				52	//812Hz we don't analyze after this index to not use resources for nothing

//semaphore
static BSEMAPHORE_DECL(start_sem, TRUE);
//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];

static TO_DO next;
static float max_norm;// sent to computer



/*
*	Wrapper to call a very optimized fft function provided by ARM
*	which uses a lot of tricks to optimize the computations
*/
void doFFT_optimized(uint16_t size, float* complex_buffer){
	if(size == FFT_SIZE)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);
}

/*
*	This function detects the index of the highest value in a buffer
*	and then depending on it, put in the variable "next" the instruction to do at the next obstacle.
*	If none of the provided frequencies is heard, we stay with the previous instruction.
*/
void sound_instruction(float* data){

	max_norm = MIN_VALUE_THRESHOLD;
	int8_t max_norm_index = -1;
	static TO_DO ancien;

	//search for the highest peak
	for(int8_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	//START
	if((max_norm_index >= (FREQ_START-1)) && (max_norm_index <= (FREQ_START+1))){
		next=START;
		chBSemSignal(&start_sem);
	}
	//turn right
	else if((max_norm_index >= (FREQ_RIGHT-1)) && (max_norm_index <= (FREQ_RIGHT+1))){
		next=TURN_RIGHT;
	}
	//turn left
	else if((max_norm_index >= (FREQ_LEFT-1)) && (max_norm_index <= (FREQ_LEFT+1))){
		next=TURN_LEFT;
	}
	//roundabout exit 1
	else if((max_norm_index >= (FREQ_RONDPOINT_EXIT1-1)) && (max_norm_index <= (FREQ_RONDPOINT_EXIT1+1))){
		next=RONDPOINT_EXIT1;
	}
	//roundabout exit 2
	else if((max_norm_index >= (FREQ_RONDPOINT_EXIT2-1)) && (max_norm_index <= (FREQ_RONDPOINT_EXIT2+1))){
		next=RONDPOINT_EXIT2;
	}
	//roundabout exit 3
	else if((max_norm_index >= (FREQ_RONDPOINT_EXIT3-1)) && (max_norm_index <= (FREQ_RONDPOINT_EXIT3+1))){
		next=RONDPOINT_EXIT3;
	}
	//roundabout exit 4
	else if((max_norm_index >= (FREQ_RONDPOINT_EXIT4-1)) && (max_norm_index <= (FREQ_RONDPOINT_EXIT4+1))){
		next=RONDPOINT_EXIT4;
	}
	//park
	else if((max_norm_index >= (FREQ_PARK-1)) && (max_norm_index <= (FREQ_PARK+1))){
		next=PARK;
	}
	else{
	//if none of the above instructions is taken, we stay with the old one
		next=ancien;
	}
	ancien=next;
}

/*
*	@brief: Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	@params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];

		nb_samples++;

		micLeft_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT processing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);


		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

		nb_samples = 0;

		sound_instruction(micLeft_output);
	}
}
/*
 * Waits until a start signal (FREQ_START sound) is detected
 * Semaphore's status modified in the function sound_instruction.
 */
void wait_start_signal(void){
	chBSemWait(&start_sem);
}


/*
*	Returns the variable "next" which indicates the instruction to do at the next obstacle.
*	This variable is modified in the function sound_instruction.
*/
TO_DO get_next_instruction(void){
	return next;
}

/*
*	@brief: Depending on the parameter instruction_aller returns what to do at the next obstacle.
*
*	@param TO_DO instruction_aller : indicates the instruction taken in front of an obstacle in that same position on the way out.
*/
TO_DO wayback_instruction(TO_DO instruction_aller){
	if (instruction_aller==TURN_RIGHT){
		return TURN_LEFT;
	}
	else if (instruction_aller==TURN_LEFT){
		return TURN_RIGHT;
	}
	else if (instruction_aller==RONDPOINT_EXIT1){
		return RONDPOINT_EXIT3;
	}
	else if (instruction_aller==RONDPOINT_EXIT3){
		return RONDPOINT_EXIT1;
	}
	else{
		//RONDPOINT_EXIT2 and RONDPOINT_EXIT4 stay the same
		//PARK and PASSAGE_PIETON cannot be taken as parameter instruction_aller
		return instruction_aller;
	}
}

float get_norm(void){
	return max_norm;
}
