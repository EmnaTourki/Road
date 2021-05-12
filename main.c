#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <sensors/proximity.h>
#include <spi_comm.h>
#include <camera/po8030.h>
#include <camera/dcmi_camera.h>
#include <audio/audio_thread.h>
#include <audio/play_melody.h>


#include <audio_processing.h>
#include <process_image.h>
#include <move.h>

#include <arm_math.h>


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //inits the motors
    motors_init();
    //inits and starts the distance sensor(TOF) to continuoulsy measure the distance.
    VL53L0X_start();
    // Starts the proximity measurement module
    proximity_start();
    //starts SPI communication thread.
    spi_comm_start();
    //starts the camera
    dcmi_start();
    po8030_start();
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);
    //starts melody's thread and the DAC module.
    dac_start();
    playMelodyStart();
    //starts the thread for the processing of the image
    process_image_start();
    //starts the movement thread
    movement_start();



    /* Infinite loop. */
    while (1) {
    	chThdSleepMilliseconds(1000);

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
