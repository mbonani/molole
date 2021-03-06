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

/** \addtogroup i2c */
/*@{*/

/** \file
	Implementation of the wrapper around I2C master.
*/

//------------
// Definitions
//------------

#include "i2c.h"
#include "i2c_priv.h"

#include "../types/uc.h"
#include "../error/error.h"
#include "../clock/clock.h"


//-----------------------
// Structures definitions
//-----------------------

/** I2C master wrapper data */
static struct 
{
	i2c_master_operation_completed_callback operation_completed_callback;	/**< function to call upon low-level operation termination */
	void* user_data; /**< optional user data that is passed to operation_completed_callback callback */
	int prev_operation; /**< previous operation, \ref I2C_MASTER_NONE if start of message */
	unsigned char* prev_data; /**< pointer to data for previous operation, in case of \ref I2C_MASTER_READ */
} I2C_Master_Data[3] = {
	{ NULL, NULL, I2C_MASTER_NONE, NULL},
	{ NULL, NULL, I2C_MASTER_NONE, NULL},
	{ NULL, NULL, I2C_MASTER_NONE, NULL}
};


//-------------------
// Exported functions
//-------------------

/**
	Init I2C master subsystem.

	\param	i2c_id
			identifier of the I2C, \ref I2C_1 or \ref I2C_2
	\param	speed
			I2C line speed in bps
	\param 	priority
			Interrupt priority, from 1 (lowest priority) to 6 (highest normal priority)
*/
void i2c_init_master(int i2c_id, long speed, int priority)
{
	unsigned long brg;
	
	i2c_check_range(i2c_id);
	ERROR_CHECK_RANGE(priority, 1, 7, GENERIC_ERROR_INVALID_INTERRUPT_PRIORITY);
#if defined __dsPIC33F__
	brg = (clock_get_cycle_frequency()/speed - clock_get_cycle_frequency()/1111111) - 1;
	ERROR_CHECK_RANGE(brg, 1, 65535, I2C_INVALID_CLOCK);
#elif defined __PIC24F__
	brg = (clock_get_cycle_frequency()/speed - clock_get_cycle_frequency()/10000000l) - 1;
	ERROR_CHECK_RANGE(brg, 1, 511, I2C_INVALID_CLOCK);
#else
	brg = 0;
	ERROR(I2C_INVALID_CLOCK, "Unsupported architecture");
#endif

	if (i2c_id == I2C_1)
	{
		I2C1BRG = (unsigned int) brg;
		_MI2C1IF = 0;					// clear the master interrupt
		_MI2C1IP = priority;			// set the master interrupt priority
	
		_MI2C1IE = 1;					// enable the master interrupt*/
	}
#ifdef _MI2C2IF
	else if (i2c_id == I2C_2)
	{
		I2C2BRG = (unsigned int) brg;
		_MI2C2IF = 0;					// clear the master interrupt
		_MI2C2IP = priority;			// set the master interrupt priority
	
		_MI2C2IE = 1;					// enable the master interrupt*/
	}
#endif
#ifdef _MI2C3IF
	else if (i2c_id == I2C_3)
	{
		I2C3BRG = (unsigned int) brg;
		_MI2C3IF = 0;					// clear the master interrupt
		_MI2C3P = priority;			// set the master interrupt priority
	
		_MI2C3IE = 1;					// enable the master interrupt*/
	}
#endif
}

/**
	Init I2C master subsystem.

	\param	i2c_id
			identifier of the I2C, \ref I2C_1 or \ref I2C_2
	\param	operation_completed_callback
			callback to high-level I2C layer on I2C interrupt
	\param 	user_data
			optional user data that is passed to operation_completed_callback callback
*/
void i2c_master_start_operations(int i2c_id, i2c_master_operation_completed_callback operation_completed_callback, void* user_data)
{
	i2c_check_range(i2c_id);
	if (I2C_Master_Data[i2c_id].operation_completed_callback != 0)
		ERROR(I2C_ERROR_MASTER_BUSY, &(I2C_Master_Data[i2c_id].operation_completed_callback));

	I2C_Master_Data[i2c_id].operation_completed_callback = operation_completed_callback;
	I2C_Master_Data[i2c_id].user_data = user_data;
	I2C_Master_Data[i2c_id].prev_operation = I2C_MASTER_NONE;

	if (i2c_id == I2C_1)
	{
		I2C1CONbits.SEN = 1;
	}
#ifdef _MI2C2IF
	else if (i2c_id == I2C_2)
	{
		I2C2CONbits.SEN = 1;
	}
#endif
#ifdef _MI2C3IF
	else if (i2c_id == I2C_3)
	{
		I2C3CONbits.SEN = 1;
	}
#endif
}

/**
	Return whether a specific I2C master is busy.

	\param	i2c_id
			identifier of the I2C, \ref I2C_1 or \ref I2C_2

	\return	true if master is busy, false otherwise
*/
bool i2c_master_is_busy(int i2c_id)
{
	i2c_check_range(i2c_id);

	return I2C_Master_Data[i2c_id].operation_completed_callback != 0;
}

/**
	Force the state machine to become idle
	This is safe to call only after a stop has been generated.
	It only reset the software state machine, not the hardware (dspic) one

	\param 	i2c_id
			identifier of the I2C, \ref I2C_1 or \ref I2C_2
*/

void i2c_master_reset(int i2c_id)
{
	i2c_check_range(i2c_id);

	I2C_Master_Data[i2c_id].operation_completed_callback = 0;
}

//--------------------------
// Interrupt service routine
//--------------------------

/**
	I2C 1 Interrupt Service Routine.
 
	Call the user-defined function.
*/
void _ISR _MI2C1Interrupt(void)
{
	unsigned char* data;
	int next_op;

	_MI2C1IF = 0;			// clear master interrupt flag

	if (I2C_Master_Data[I2C_1].prev_operation == I2C_MASTER_READ)
		*(I2C_Master_Data[I2C_1].prev_data) = I2C1RCV;

	next_op = I2C_Master_Data[I2C_1].operation_completed_callback(I2C_1, &data, I2C_Master_Data[I2C_1].user_data, I2C1STATbits.ACKSTAT);

	switch (next_op)
	{
		case I2C_MASTER_READ:
			I2C_Master_Data[I2C_1].prev_data = data;
			I2C1CONbits.RCEN = 1;
		break;

		case I2C_MASTER_WRITE:
			I2C1TRN = *data;
		break;

		case I2C_MASTER_RESTART:
			I2C1CONbits.RSEN = 1;
		break;

		case I2C_MASTER_ACK:
			I2C1CONbits.ACKDT = 0;
			I2C1CONbits.ACKEN = 1;
		break;

		case I2C_MASTER_NACK:
			I2C1CONbits.ACKDT = 1;
			I2C1CONbits.ACKEN = 1;
		break;

		case I2C_MASTER_STOP:
			I2C1CONbits.PEN = 1;
		break;

		case I2C_MASTER_DONE:
			I2C_Master_Data[I2C_1].operation_completed_callback = 0;
		break;
		
		case I2C_MASTER_QUIT:
			return;

		default:
			ERROR(I2C_INVALID_OPERATION, &next_op);
		break;
	}

	I2C_Master_Data[I2C_1].prev_operation = next_op;
}


/**
	I2C 2 Interrupt Service Routine.
 
	Call the user-defined function.
*/
#ifdef _MI2C2IF
void _ISR _MI2C2Interrupt(void)
{
	unsigned char* data;
	int next_op;

	_MI2C2IF = 0;			// clear master interrupt flag

	if (I2C_Master_Data[I2C_2].prev_operation == I2C_MASTER_READ)
		*(I2C_Master_Data[I2C_2].prev_data) = I2C2RCV;

	next_op = I2C_Master_Data[I2C_2].operation_completed_callback(I2C_2, &data, I2C_Master_Data[I2C_2].user_data, I2C2STATbits.ACKSTAT);

	switch (next_op)
	{
		case I2C_MASTER_READ:
			I2C_Master_Data[I2C_2].prev_data = data;
			I2C2CONbits.RCEN = 1;
		break;

		case I2C_MASTER_WRITE:
			I2C2TRN = *data;
		break;

		case I2C_MASTER_RESTART:
			I2C2CONbits.RSEN = 1;
		break;

		case I2C_MASTER_ACK:
			I2C2CONbits.ACKDT = 0;
			I2C2CONbits.ACKEN = 1;
		break;

		case I2C_MASTER_NACK:
			I2C2CONbits.ACKDT = 1;
			I2C2CONbits.ACKEN = 1;
		break;

		case I2C_MASTER_STOP:
			I2C2CONbits.PEN = 1;
		break;

		case I2C_MASTER_DONE:
			I2C_Master_Data[I2C_2].operation_completed_callback = 0;
		break;
		
		case I2C_MASTER_QUIT:
			return;

		default:
			ERROR(I2C_INVALID_OPERATION, &next_op);
		break;
	}

	I2C_Master_Data[I2C_2].prev_operation = next_op;
}
#endif

/**
	I2C 3 Interrupt Service Routine.
 
	Call the user-defined function.
*/
#ifdef _MI2C3IF
void _ISR _MI2C3Interrupt(void)
{
	unsigned char* data;
	int next_op;

	_MI2C3IF = 0;			// clear master interrupt flag

	if (I2C_Master_Data[I2C_3].prev_operation == I2C_MASTER_READ)
		*(I2C_Master_Data[I2C_3].prev_data) = I2C3RCV;

	next_op = I2C_Master_Data[I2C_3].operation_completed_callback(I2C_3, &data, I2C_Master_Data[I2C_3].user_data, I2C3STATbits.ACKSTAT);

	switch (next_op)
	{
		case I2C_MASTER_READ:
			I2C_Master_Data[I2C_3].prev_data = data;
			I2C3CONbits.RCEN = 1;
		break;

		case I2C_MASTER_WRITE:
			I2C3TRN = *data;
		break;

		case I2C_MASTER_RESTART:
			I2C3CONbits.RSEN = 1;
		break;

		case I2C_MASTER_ACK:
			I2C3CONbits.ACKDT = 0;
			I2C3CONbits.ACKEN = 1;
		break;

		case I2C_MASTER_NACK:
			I2C3CONbits.ACKDT = 1;
			I2C3CONbits.ACKEN = 1;
		break;

		case I2C_MASTER_STOP:
			I2C3CONbits.PEN = 1;
		break;

		case I2C_MASTER_DONE:
			I2C_Master_Data[I2C_3].operation_completed_callback = 0;
		break;
		
		case I2C_MASTER_QUIT:
			return;

		default:
			ERROR(I2C_INVALID_OPERATION, &next_op);
		break;
	}

	I2C_Master_Data[I2C_3].prev_operation = next_op;
}
#endif
/*@}*/
