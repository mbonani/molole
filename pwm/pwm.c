/*
	Molole - Mobots Low Level library
	An open source toolkit for robot programming using DsPICs

	Copyright (C) 2007--2011 Stephane Magnenat <stephane at magnenat dot net>,
	Philippe Retornaz <philippe dot retornaz at epfl dot ch>
	Mobots group (http://mobots.epfl.ch), Robotics system laboratory (http://lsro.epfl.ch)
	EPFL Ecole polytechnique federale de Lausanne (http://www.epfl.ch)

	See authors.txt for more details about other contributors.

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published
	by the Free Software Foundation, version 3 of the License.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

//--------------------
// Usage documentation
//--------------------

/**
	\defgroup pwm PWM
	
	Wrapper around PWM, with a callback oriented interface.
*/
/*@{*/

/** \file
	Implementation of the wrapper around PWM.
*/


//------------
// Definitions
//------------

#include <p33Fxxxx.h>

#include "pwm.h"
#include "../error/error.h"

/** PWM wrapper data */
static struct
{
	pwm_callback interrupt_callback; /**< function to call upon PWM interrupt */
	int mode[4];					 /**< mode */
	int inverted[4];				 /**< state of the PMOD bbit in PWMxCON1 */
	unsigned char reverse[4];			 /**< reverse the sens */
	unsigned int period;
} PWM_Data;

/**
	Init the PWM subsystem.
	
	\param	prescaler
			Prescaler of FCY. Must be one of \ref pwm_prescaler_values .
	\param	period
			PWM period (0..32767)
	\param	mode
			PWM time base modes. Must be one of \ref pwm_time_base_modes .
*/
void pwm_init(int prescaler, unsigned period, int mode)
{
	int i;
	ERROR_CHECK_RANGE(prescaler, 0, 3, PWM_ERROR_INVALID_PRESCALER);
	ERROR_CHECK_RANGE(period, 0, 32767, PWM_ERROR_INVALID_RANGE);
	ERROR_CHECK_RANGE(mode, 0, 3, PWM_ERROR_INVALID_MODE);
	
	PTPER = period;						// PWM period
	PTCONbits.PTCKPS = prescaler;		// PWM Time Base Input Clock Prescale
	PTCONbits.PTMOD = mode;				// PWM Time Base operates in continuous up/down counting mode
	PTCONbits.PTSIDL = 0;				// PWM time base runs in CPU Idle mode
	PTCONbits.PTEN = 1;					// Enable PWM Time Base Timer
	DTCON1 = 0;							// Disable any dead time generator
	DTCON2 = 0;
	
	for(i = 0; i < 4; i++) 
		PWM_Data.inverted[i] = 1; // Independant mode by default 
	
	switch(mode) {
		case PWM_MODE_FREE_RUNNING:
		case PWM_MODE_SINGLE_EVENT:
			PWM_Data.period = period;
			break;
		case PWM_CONTINUOUS_UP_DOWN:
		case PWM_CONTINUOUS_UP_DOWN_DOUBLE:
			PWM_Data.period = period << 1;
			break;
	}
}

/**
	Enable the PWM interrupt.
	
	\param	postscaler
			The conversion is started each (postscale+1) (parameter is 0..15, corresponding to a 1:1 to 1:16 postscale)
	\param	callback
			Pointer to a function that will be called upon interrupt
	\param 	priority
			Interrupt priority, from 1 (lowest priority) to 6 (highest normal priority)
*/
void pwm_enable_interrupt(int postscaler, pwm_callback callback, int priority)
{
	ERROR_CHECK_RANGE(postscaler, 0, 15, PWM_ERROR_INVALID_POSTSCALER);
	ERROR_CHECK_RANGE(priority, 1, 7, GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY);
	
	PTCONbits.PTOPS = postscaler;		// PWM Time Base Output Postscale
	
	PWM_Data.interrupt_callback = callback;
	_PWMIF = 0;							// clear the PWM interrupt
	_PWMIP = priority;   				// set the PWM interrupt priority
	_PWMIE = 1;							// enable the PWM interrupt
}

/**
	Disable the PWM interrupt
*/
void pwm_disable_interrupt()
{
	_PWMIE = 0;							// disable the PWM interrupt
	_PWMIF = 0;							// clear the PWM interrupt
}

/**
	Disables a PWM.
	
	\param	pwm_id
			Identifier of the PWM, from \ref PWM_1 to \ref PWM_4 .
*/
void pwm_disable(int pwm_id)
{
	// L and H pins are to GPIO
	PWMCON2bits.UDIS = 1;
	switch (pwm_id)
	{
		case PWM_1: PWMCON1bits.PEN1L = 0; PWMCON1bits.PEN1H = 0; break;
		case PWM_2: PWMCON1bits.PEN2L = 0; PWMCON1bits.PEN2H = 0; break;
		case PWM_3: PWMCON1bits.PEN3L = 0; PWMCON1bits.PEN3H = 0; break;
		case PWM_4: PWMCON1bits.PEN4L = 0; PWMCON1bits.PEN4H = 0; break;
		default: ERROR(PWM_ERROR_INVALID_PWM_ID, &pwm_id);
	}
	PWMCON2bits.UDIS = 0;
}


/**
	Set the duty of a PWM. Implicitly enable a PWM output.
	
	\param	pwm_id
			Identifier of the PWM, from \ref PWM_1 to \ref PWM_4 .
	\param	duty
			Duty cycle (0..32767)
*/

void pwm_set_duty(int pwm_id, int duty)
{

	PWMCON2bits.UDIS = 1;
	switch (pwm_id)
	{
		case PWM_1:
			if(PWM_Data.reverse[pwm_id])
				duty = -duty;
				
			PWMCON1bits.PEN1L = 1; 
			PWMCON1bits.PEN1H = 1; 
			PWMCON1bits.PMOD1 = PWM_Data.inverted[pwm_id]; 

			if(duty == 0) {
				OVDCONbits.POVD1L = 0;
				OVDCONbits.POVD1H = 0;
				switch(PWM_Data.mode[pwm_id]) {
				case PWM_ONE_DEFAULT_LOW:
				case PWM_BOTH_DEFAULT_LOW:
					OVDCONbits.POUT1L = 0;
					OVDCONbits.POUT1H = 0;
					break;
				case PWM_ONE_DEFAULT_HIGH:
				case PWM_BOTH_DEFAULT_HIGH:
					OVDCONbits.POUT1L = 1;
					OVDCONbits.POUT1H = 1;
					break;
				}
			} else if(duty > 0) {
				/* PWM1L is forced low */
				switch(PWM_Data.mode[pwm_id]) {
				case PWM_ONE_DEFAULT_LOW:
					OVDCONbits.POUT1L = 0;
					OVDCONbits.POVD1L = 0;
					OVDCONbits.POVD1H = 1;
					break;
				case PWM_BOTH_DEFAULT_LOW:
					OVDCONbits.POVD1H = 1;
					OVDCONbits.POVD1L = 1;
					break;
				case PWM_ONE_DEFAULT_HIGH:
					OVDCONbits.POUT1L = 1;
					OVDCONbits.POVD1L = 0;
					OVDCONbits.POVD1H = 1;
					if(duty > PWM_Data.period) 
						duty = PWM_Data.period;
					duty = PWM_Data.period - duty;
					break;
				case PWM_BOTH_DEFAULT_HIGH:
					OVDCONbits.POVD1L = 1;
					OVDCONbits.POVD1H = 1;
					if(duty > PWM_Data.period) 
						duty = PWM_Data.period;
					duty = PWM_Data.period - duty;
					break;
				}
		
				PDC1 = duty; 
			} else {
				switch(PWM_Data.mode[pwm_id]) {
				case PWM_ONE_DEFAULT_LOW:
					OVDCONbits.POUT1H = 0;
					OVDCONbits.POVD1H = 0;
					OVDCONbits.POVD1L = 1;
					break;
				case PWM_BOTH_DEFAULT_LOW:
					OVDCONbits.POVD1H = 1;
					OVDCONbits.POVD1L = 1;
					break;
				case PWM_ONE_DEFAULT_HIGH:
					OVDCONbits.POUT1H = 1;
					OVDCONbits.POVD1H = 0;
					OVDCONbits.POVD1L = 1;
					if(duty < -PWM_Data.period) 
						duty = -PWM_Data.period;
					duty = -PWM_Data.period - duty;
					break;
				case PWM_BOTH_DEFAULT_HIGH:
					OVDCONbits.POVD1L = 1;
					OVDCONbits.POVD1H = 1;
					if(duty < -PWM_Data.period) 
						duty = -PWM_Data.period;
					duty = -PWM_Data.period - duty;
					break;
				}
		
				PDC1 = -duty; 
			}
				
			break;
		case PWM_2:
		
			if(PWM_Data.reverse[pwm_id])
				duty = -duty;
		
			PWMCON1bits.PEN2L = 1; 
			PWMCON1bits.PEN2H = 1; 
			PWMCON1bits.PMOD2 = PWM_Data.inverted[pwm_id]; 

			if(duty == 0) {
				OVDCONbits.POVD2L = 0;
				OVDCONbits.POVD2H = 0;
				switch(PWM_Data.mode[pwm_id]) {
				case PWM_ONE_DEFAULT_LOW:
				case PWM_BOTH_DEFAULT_LOW:
					OVDCONbits.POUT2L = 0;
					OVDCONbits.POUT2H = 0;
					break;
				case PWM_ONE_DEFAULT_HIGH:
				case PWM_BOTH_DEFAULT_HIGH:
					OVDCONbits.POUT2L = 1;
					OVDCONbits.POUT2H = 1;
					break;
				}
			} else if(duty > 0) {
				/* PWM1L is forced low */
				switch(PWM_Data.mode[pwm_id]) {
				case PWM_ONE_DEFAULT_LOW:
					OVDCONbits.POUT2L = 0;
					OVDCONbits.POVD2L = 0;
					OVDCONbits.POVD2H = 1;
					break;
				case PWM_BOTH_DEFAULT_LOW:
					OVDCONbits.POVD2H = 1;
					OVDCONbits.POVD2L = 1;
					break;
				case PWM_ONE_DEFAULT_HIGH:
					OVDCONbits.POUT2L = 1;
					OVDCONbits.POVD2L = 0;
					OVDCONbits.POVD2H = 1;
					if(duty > PWM_Data.period) 
						duty = PWM_Data.period;
					duty = PWM_Data.period - duty;
					break;
				case PWM_BOTH_DEFAULT_HIGH:
					OVDCONbits.POVD2L = 1;
					OVDCONbits.POVD2H = 1;
					if(duty > PWM_Data.period) 
						duty = PWM_Data.period;
					duty = PWM_Data.period - duty;
					break;
				}
		
				PDC2 = duty; 
			} else {
				switch(PWM_Data.mode[pwm_id]) {
				case PWM_ONE_DEFAULT_LOW:
					OVDCONbits.POUT2H = 0;
					OVDCONbits.POVD2H = 0;
					OVDCONbits.POVD2L = 1;
					break;
				case PWM_BOTH_DEFAULT_LOW:
					OVDCONbits.POVD2H = 1;
					OVDCONbits.POVD2L = 1;
					break;
				case PWM_ONE_DEFAULT_HIGH:
					OVDCONbits.POUT2H = 1;
					OVDCONbits.POVD2H = 0;
					OVDCONbits.POVD2L = 1;
					if(duty < -PWM_Data.period) 
						duty = -PWM_Data.period;
					duty = -PWM_Data.period - duty;
					break;
				case PWM_BOTH_DEFAULT_HIGH:
					OVDCONbits.POVD2L = 1;
					OVDCONbits.POVD2H = 1;
					if(duty < -PWM_Data.period) 
						duty = -PWM_Data.period;
					duty = -PWM_Data.period - duty;
					break;
				}
		
				PDC2 = -duty; 
			}
				
			break;
		case PWM_3:
			if(PWM_Data.reverse[pwm_id])
				duty = -duty;
				
			PWMCON1bits.PEN3L = 1; 
			PWMCON1bits.PEN3H = 1; 
			PWMCON1bits.PMOD3 = PWM_Data.inverted[pwm_id]; 

			if(duty == 0) {
				OVDCONbits.POVD3L = 0;
				OVDCONbits.POVD3H = 0;
				switch(PWM_Data.mode[pwm_id]) {
				case PWM_ONE_DEFAULT_LOW:
				case PWM_BOTH_DEFAULT_LOW:
					OVDCONbits.POUT3L = 0;
					OVDCONbits.POUT3H = 0;
					break;
				case PWM_ONE_DEFAULT_HIGH:
				case PWM_BOTH_DEFAULT_HIGH:
					OVDCONbits.POUT3L = 1;
					OVDCONbits.POUT3H = 1;
					break;
				}
			} else if(duty > 0) {
				/* PWM1L is forced low */
				switch(PWM_Data.mode[pwm_id]) {
				case PWM_ONE_DEFAULT_LOW:
					OVDCONbits.POUT3L = 0;
					OVDCONbits.POVD3L = 0;
					OVDCONbits.POVD3H = 1;
					break;
				case PWM_BOTH_DEFAULT_LOW:
					OVDCONbits.POVD3H = 1;
					OVDCONbits.POVD3L = 1;
					break;
				case PWM_ONE_DEFAULT_HIGH:
					OVDCONbits.POUT3L = 1;
					OVDCONbits.POVD3L = 0;
					OVDCONbits.POVD3H = 1;
					if(duty > PWM_Data.period) 
						duty = PWM_Data.period;
					duty = PWM_Data.period - duty;
					break;
				case PWM_BOTH_DEFAULT_HIGH:
					OVDCONbits.POVD3L = 1;
					OVDCONbits.POVD3H = 1;
					if(duty > PWM_Data.period) 
						duty = PWM_Data.period;
					duty = PWM_Data.period - duty;
					break;
				}
		
				PDC3 = duty; 
			} else {
				switch(PWM_Data.mode[pwm_id]) {
				case PWM_ONE_DEFAULT_LOW:
					OVDCONbits.POUT3H = 0;
					OVDCONbits.POVD3H = 0;
					OVDCONbits.POVD3L = 1;
					break;
				case PWM_BOTH_DEFAULT_LOW:
					OVDCONbits.POVD3H = 1;
					OVDCONbits.POVD3L = 1;
					break;
				case PWM_ONE_DEFAULT_HIGH:
					OVDCONbits.POUT3H = 1;
					OVDCONbits.POVD3H = 0;
					OVDCONbits.POVD3L = 1;
					if(duty < -PWM_Data.period) 
						duty = -PWM_Data.period;
					duty = -PWM_Data.period - duty;
					break;
				case PWM_BOTH_DEFAULT_HIGH:
					OVDCONbits.POVD3L = 1;
					OVDCONbits.POVD3H = 1;
					if(duty < -PWM_Data.period) 
						duty = -PWM_Data.period;
					duty = -PWM_Data.period - duty;
					break;
				}
		
				PDC3 = -duty; 
			}
				
			break;
		case PWM_4:
			if(PWM_Data.reverse[pwm_id])
				duty = -duty;
				
			PWMCON1bits.PEN4L = 1; 
			PWMCON1bits.PEN4H = 1; 
			PWMCON1bits.PMOD4 = PWM_Data.inverted[pwm_id]; 

			if(duty == 0) {
				OVDCONbits.POVD4L = 0;
				OVDCONbits.POVD4H = 0;
				switch(PWM_Data.mode[pwm_id]) {
				case PWM_ONE_DEFAULT_LOW:
				case PWM_BOTH_DEFAULT_LOW:
					OVDCONbits.POUT4L = 0;
					OVDCONbits.POUT4H = 0;
					break;
				case PWM_ONE_DEFAULT_HIGH:
				case PWM_BOTH_DEFAULT_HIGH:
					OVDCONbits.POUT4L = 1;
					OVDCONbits.POUT4H = 1;
					break;
				}
			} else if(duty > 0) {
				/* PWM1L is forced low */
				switch(PWM_Data.mode[pwm_id]) {
				case PWM_ONE_DEFAULT_LOW:
					OVDCONbits.POUT4L = 0;
					OVDCONbits.POVD4L = 0;
					OVDCONbits.POVD4H = 1;
					break;
				case PWM_BOTH_DEFAULT_LOW:
					OVDCONbits.POVD4H = 1;
					OVDCONbits.POVD4L = 1;
					break;
				case PWM_ONE_DEFAULT_HIGH:
					OVDCONbits.POUT4L = 1;
					OVDCONbits.POVD4L = 0;
					OVDCONbits.POVD4H = 1;
					if(duty > PWM_Data.period) 
						duty = PWM_Data.period;
					duty = PWM_Data.period - duty;
					break;
				case PWM_BOTH_DEFAULT_HIGH:
					OVDCONbits.POVD4L = 1;
					OVDCONbits.POVD4H = 1;
					if(duty > PWM_Data.period) 
						duty = PWM_Data.period;
					duty = PWM_Data.period - duty;
					break;
				}
		
				PDC4 = duty; 
			} else {
				switch(PWM_Data.mode[pwm_id]) {
				case PWM_ONE_DEFAULT_LOW:
					OVDCONbits.POUT4H = 0;
					OVDCONbits.POVD4H = 0;
					OVDCONbits.POVD4L = 1;
					break;
				case PWM_BOTH_DEFAULT_LOW:
					OVDCONbits.POVD4H = 1;
					OVDCONbits.POVD4L = 1;
					break;
				case PWM_ONE_DEFAULT_HIGH:
					OVDCONbits.POUT4H = 1;
					OVDCONbits.POVD4H = 0;
					OVDCONbits.POVD4L = 1;
					if(duty < -PWM_Data.period) 
						duty = -PWM_Data.period;
					duty = -PWM_Data.period - duty;
					break;
				case PWM_BOTH_DEFAULT_HIGH:
					OVDCONbits.POVD4L = 1;
					OVDCONbits.POVD4H = 1;
					if(duty < -PWM_Data.period) 
						duty = -PWM_Data.period;
					duty = -PWM_Data.period - duty;
					break;
				}
		
				PDC4 = -duty; 
			}
				
			break;
		default: ERROR(PWM_ERROR_INVALID_PWM_ID, &pwm_id);
	}
	PWMCON2bits.UDIS = 0;
}

/**
	Setup analog-to-digital conversion (adc) synchronization with PWM.
	
	When PWM reaches a specific value and direction, a counter is incremented.
	When this counter reaches (postscale+1), the adc starts.
	
	This function does not perform adc setup.
	
	\param	direction
			On which direction of counting to compare, \ref PWM_SEV_UP or \ref PWM_SEV_DOWN
	\param	postscale
			The conversion is started each (postscale+1) (parameter is 0..15, corresponding to a 1:1 to 1:16 postscale)
	\param	value
			Value to compare (0.. 32767)
*/
void pwm_set_special_event_trigger(int direction, int postscale, unsigned value)
{
	ERROR_CHECK_RANGE(direction, 0, 1, PWM_ERROR_INVALID_SEV_DIRECTION);
	ERROR_CHECK_RANGE(postscale, 0, 15, PWM_ERROR_INVALID_SEV_POSTSCALE);
	ERROR_CHECK_RANGE(value, 0, 32767, PWM_ERROR_INVALID_RANGE);
	
	SEVTCMPbits.SEVTDIR = direction;
	SEVTCMPbits.SEVTCMP = value;
	PWMCON2bits.SEVOPS = postscale;
	
	// TODO: set or check ADC, for instance: AD1CON1bits.SSRC
}

void pwm_set_brake(int pwm_id, int mode)
{
	ERROR_CHECK_RANGE(pwm_id, 0, 3, PWM_ERROR_INVALID_PWM_ID);
	ERROR_CHECK_RANGE(mode, 0, 6, PWM_ERROR_INVALID_MODE);
	
	if(mode == PWM_BOTH_INVERTED_DEFAULT_LOW) {
		mode = PWM_BOTH_DEFAULT_LOW;
		PWM_Data.inverted[pwm_id] = 0;
	} else if ( mode == PWM_BOTH_INVERTED_DEFAULT_HIGH) {
		mode = PWM_BOTH_DEFAULT_HIGH;
		PWM_Data.inverted[pwm_id] = 0;
	} else {
		PWM_Data.inverted[pwm_id] = 1;
	
	}
	PWM_Data.mode[pwm_id] = mode;
}

void pwm_invert(int pwm_id, int invert) 
{
	ERROR_CHECK_RANGE(pwm_id, 0, 3, PWM_ERROR_INVALID_PWM_ID);
	PWM_Data.reverse[pwm_id] = invert;
}
	
//--------------------------
// Interrupt service routine
//--------------------------

/**
	PWM Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _PWMInterrupt(void)
{
	_PWMIF = 0;

	PWM_Data.interrupt_callback();
}

/*@}*/
