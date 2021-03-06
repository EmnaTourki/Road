//amended from tp4_Camreg

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <camera/dcmi_camera.h>
#include <process_image.h>

#define IMAGE_BUFFER_SIZE					640		//pixels
#define WIDTH_SLOPE							5		//pixels
#define MIN_LINE_WIDTH						40		//pixels
#define PASSAGE_PIETON_LINE_WIDTH			200		//pixels
#define COUNTER_MAX_VALUE					10
#define COUNTER_FIN_MAX_VALUE				50
#define BUFFER_LINE							280

static uint16_t lineWidth = 0;


//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint16_t extract_line_width(uint8_t *buffer){

	uint16_t i = 0, width = 0 ,begin = 0, end = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	//performs an average
	for(uint16_t i =0 ; i <IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
		    {
		        begin = i;
		        stop = 1;
		        i=i+WIDTH_SLOPE;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if ((i < IMAGE_BUFFER_SIZE) && begin)
		{
		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if ((i >= IMAGE_BUFFER_SIZE) && (!end))
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}while(wrong_line);

	if(line_not_found){
		width = 0;
	}else{
		width = (end - begin);
	}
	return width;
}

static THD_WORKING_AREA(waCaptureImage, 512);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 280 + 281 to see the ground and eventually the pedestrian crossing
	po8030_advanced_config(FORMAT_RGB565, 0, BUFFER_LINE, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 2048);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	lineWidth = 0;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		//search for a line in the image and gets its width in pixels
		lineWidth = extract_line_width(image);


    }
}

/*
 *  If lineWidth exceeds PASSAGE_PIETON_LINE_WIDTH COUNTER_MAX_VALUE times , there is a pedestrian crossing and the function returns true
 *  If not returns false
 */
bool passage_pieton(void){
	static uint8_t counter=0;
	if (lineWidth>=PASSAGE_PIETON_LINE_WIDTH){
		counter++;
		if(counter>=COUNTER_MAX_VALUE){
			counter=0;
			return true;
		}else return false;
	}else{
		counter=0;
		return false;
	}
}

/*
 * If no line is detected COUNTER_MAX_VALUE times, we crossed the pedestrian crossing and the function returns true
 */
bool fin_passage_pieton(void){
	static uint8_t counter=0;
	if(lineWidth==0){
		counter++;
		if(counter>=COUNTER_FIN_MAX_VALUE){
			counter=0;
			return true;
		}else return false;
	}else{
		counter=0;
		return false;
	}
}
/*
 * Returns the width (in pixels) of a line calculated in the function extract_line_width
 */
uint16_t get_lineWidth(void){
	return lineWidth;
}


//start the threads
void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
