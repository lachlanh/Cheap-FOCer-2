/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "mc_interface.h"
#include "utils.h"
#include "encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 2048);

//pas config
//Pedal Assist constants
const int CADENCE_MAGNETS = 12;
const int CADENCE_MIN = 15; // minimum cadence for motor to run
const int CADENCE_MAX = 45; // cadence value that will result in full throttle

const float THROTTLE_OFF = 0.0;
const float THROTTLE_DUTY_MIN = 0.05;
const float THROTTLE_DUTY_MID = 0.5;
const float THROTTLE_DUTY_MAX = 0.8;

const int PAS_TIMEOUT = 400; //ms to stop power after pedal stopped
#define FILTER_SAMPLES					5


// Private functions
static void pwm_callback(void);
static void terminal_test(int argc, const char **argv);
static float calculateCadence (long edgeInterval, int cadenceMagnets);
static float calculateThrottleDuty (float targetDuty, float cadence);

// Private variables
static volatile custom_config config;
static volatile bool stop_now = true;
static volatile bool is_running = false;

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
	mc_interface_set_pwm_callback(pwm_callback);

	stop_now = false;
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
			NORMALPRIO, my_thread, NULL);


        palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_INPUT_PULLUP);

        // Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"custom_cmd",
			"Print the number d",
			"[d]",
			terminal_test);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	mc_interface_set_pwm_callback(0);
	terminal_unregister_callback(terminal_test);

	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_custom_configure(custom_config *conf) {
	config = *conf;
}

static THD_FUNCTION(my_thread, arg) {
	(void)arg;

	chRegSetThreadName("App Custom PAS");

	is_running = true;
        //graph of throttleDuty setup
        commands_init_plot("Sample", "Throttle Duty");
	commands_plot_add_graph("ThrottleDuty");
        commands_plot_add_graph("ThrottleRamp");
        float samp = 0.0;

        bool pas_input = true;
        bool pas_previous = true;
        static systime_t last_time = 0;
        static systime_t elapsed_time = 0;
        static systime_t report_time = 0;
        float cadence = 0.0;
        float throttleDuty = 0.0;
        bool switchOn1 = false;
        bool switchOn3 = false;
        float targetDuty = THROTTLE_OFF;

	for(;;) {
		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}

                //switches

                float adc1 = (float)ADC_Value[ADC_IND_EXT];
                if (adc1 >= 2500) {
                  switchOn3 = true;
                } else {
                  switchOn3 = false;
                }
                float adc2 = (float)ADC_Value[ADC_IND_EXT2];
                if (adc2 >= 2500) {
                  switchOn1 = true;
                } else {
                  switchOn1 = false;
                }
                
                if (switchOn1) {
                  targetDuty = THROTTLE_OFF;
                } else if (switchOn3) {
                  targetDuty = THROTTLE_DUTY_MAX;//TODO LH looks like adc2 is always on on second focer.
                } else {
                  targetDuty = THROTTLE_DUTY_MID;
                }
                
                if (ST2MS (chVTTimeElapsedSinceX (report_time)) > 500) {
                  report_time = chVTGetSystemTimeX ();
                  //TODO try and plot this
                  commands_printf("adc1: %f adc2: %f targetDuty: %f ---", adc1, adc2, targetDuty);
                }
                
                //PAS bits
                pas_input = palReadPad(HW_ICU_GPIO, HW_ICU_PIN);
                if (pas_input != pas_previous) {
                  if (pas_input ==1) {
                    elapsed_time = ST2MS (chVTTimeElapsedSinceX (last_time));
                    last_time = chVTGetSystemTimeX ();

                    cadence = calculateCadence (elapsed_time, 12);
                    throttleDuty = calculateThrottleDuty (targetDuty, cadence);
                    
                    commands_printf ("PAS CHANGED pas_input := %d elapsed time: %d cadence: %f throttleDuty : %f", pas_input, elapsed_time, cadence, throttleDuty);
                  }
                };
                pas_previous = pas_input;

                //from adc filter the sample
                // Optionally apply a mean value filter
		//if (config.use_filter{
                static float filter_buffer[FILTER_SAMPLES];
                static int filter_ptr = 0;
                static float throttleFilter = 0.0;
                
                filter_buffer[filter_ptr++] = throttleDuty;
                if (filter_ptr >= FILTER_SAMPLES) {
                  filter_ptr = 0;
                }
                
                throttleFilter = 0.0;
                for (int i = 0;i < FILTER_SAMPLES;i++) {
                  throttleFilter += filter_buffer[i];
                }
                throttleFilter /= FILTER_SAMPLES;
                //}
                /*
                static float throttle_ramp = 0.0;
                if (throttleDuty > THROTTLE_OFF) {
                  
                  // Apply ramping
                  static systime_t last_ramp_time = 0;
                  
                  static float ramp_time = 0.4; 

                  if (ramp_time > 0.01) {
                    const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_ramp_time)) / (ramp_time * 1000.0);
                    commands_plot_set_graph(1);
                    commands_send_plot_points(samp, ramp_step);
                    if (throttle_ramp < throttleDuty) {
                      if ((throttle_ramp + ramp_step < throttleDuty)) {
                        throttle_ramp += ramp_step;
                      } else {
                        throttle_ramp = throttleDuty;
                      }
                    } else if (throttle_ramp > throttleDuty) {
                      if ((throttle_ramp - ramp_step) > throttleDuty) {
                        throttle_ramp -= ramp_step;
                      } else {
                        throttle_ramp = throttleDuty;
                      }
                    }
                    last_ramp_time = chVTGetSystemTimeX();
                    //throttleDuty = throttle_ramp;
                  }
                  
                  if (ST2MS (chVTTimeElapsedSinceX (report_time)) > 500) {
                    report_time = chVTGetSystemTimeX ();
                    //TODO try and plot this
                    commands_printf ("ramped  throttleDuty: %f throttle_ramp: %f last_time: %d", throttleDuty, throttle_ramp, last_time);
                    }
                    }*/

                
                
                if (throttleDuty == THROTTLE_OFF) {
                  mc_interface_set_current (THROTTLE_OFF);
                } else {
                  mc_interface_set_duty(throttleDuty);
                }
                timeout_reset ();

                //plot the throttleduty
                commands_plot_set_graph(0);
		commands_send_plot_points(samp, throttleDuty);
                //commands_plot_set_graph(1);
                //commands_send_plot_points(samp, throttle_ramp);
                samp++;
                
                //TODO LH need a condition so if time since last signal > some_timeout (100ms
                if (last_time != 0 && ST2MS (chVTTimeElapsedSinceX (last_time)) > 400) {
                  //check if the motor is running and set to off
                  if (mc_interface_get_duty_cycle_now() > 0.0) {
                    mc_interface_set_current (THROTTLE_OFF);
                    last_time = 0;
                    throttleDuty = 0;
                    commands_printf("PAS TIMEOUT");
                  }
                }
                
                
		chThdSleepMilliseconds(10);
	
        }
}

static float calculateThrottleDuty (float targetDuty, float cadence) {

  float throttleDuty = THROTTLE_OFF;
  float throttleStep = 0.0;

  throttleStep = (targetDuty - THROTTLE_DUTY_MIN) / (CADENCE_MAX - CADENCE_MIN);
  if (targetDuty == THROTTLE_OFF)
  {
    throttleDuty = THROTTLE_OFF;
  }
  else if (cadence > CADENCE_MAX)
  {
    throttleDuty = targetDuty;
  }
  else if (cadence < CADENCE_MIN)
  {
    throttleDuty = THROTTLE_OFF;
  }
  else
  {
    //need to start collecting these up
    throttleDuty = ((cadence - CADENCE_MIN) * throttleStep) + THROTTLE_DUTY_MIN;
  }
  return throttleDuty;
  
}

static float calculateCadence (long edgeInterval, int cadenceMagnets) {
  float cadence = 0.0;
  if (edgeInterval > 0)
  {
    //this calculation always gives multiples of 5.. think it is faulty
    //cadence = (1000 / edgeInterval) * (60 / 12); //should give rpm this is getting truncated somehow.. 

    cadence = 60000 / (edgeInterval * cadenceMagnets); 
  }
  else
  {
    cadence = 0.0;
  }
  return cadence;
}

static void pwm_callback(void) {
	// Called for every control iteration in interrupt context.
}

// Callback function for the terminal command with arguments.
static void terminal_test(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);

		commands_printf("You have entered %d", d);

		// For example, read the ADC inputs on the COMM header.
		commands_printf("ADC1: %.2f V ADC2: %.2f V",
				(double)ADC_VOLTS(ADC_IND_EXT), (double)ADC_VOLTS(ADC_IND_EXT2));
	} else {
		commands_printf("This command requires one argument.\n");
	}
}
