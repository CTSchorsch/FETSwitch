/**
 * USITWIM - USI as TWI in Master mode
 *
 * @created: 2015-02-03
 * @author: Neven Boyanov
 *
 * This is part of the Tinusaur/USITWIX project.
 *
 * Copyright (c) 2016 Neven Boyanov, Tinusaur Team. All Rights Reserved.
 * Distributed as open source software under MIT License, see LICENSE.txt file.
 * Please, as a favor, retain the link http://tinusaur.org to The Tinusaur Project.
 *
 * Source code available at: https://bitbucket.org/tinusaur/usitwix
 *
 */

#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

#include "usitwim.h"

// ============================================================================

#define USITWIM_RESULT_SUCCESS 0x00
#define USITWIM_RESULT_FAILURE 0xFF

// ----------------------------------------------------------------------------

// void USI_TWI_Master_Initialise( void );
uint8_t USI_TWI_Start_Random_Read( unsigned char * , unsigned char );
uint8_t USI_TWI_Start_Read_Write( unsigned char * , unsigned char );
uint8_t USI_TWI_Get_Error_State(void);

uint8_t USI_TWI_Start_Transceiver_With_Data(uint8_t *, uint8_t);
uint8_t USI_TWI_Master_Transfer(uint8_t);
uint8_t USI_TWI_Master_Stop(void);
uint8_t USI_TWI_Master_Start(void);

// ----------------------------------------------------------------------------

//	USI TWI single master initialization function
void usitwim_init(void)
{
	PORT_USI |= (1<<PIN_USI_SDA);			// Enable pullup on SDA, to set high as released state.
	PORT_USI |= (1<<PIN_USI_SCL);			// Enable pullup on SCL, to set high as released state.

	DDR_USI	 |= (1<<PIN_USI_SCL);			// Enable SCL as output.
	DDR_USI	 |= (1<<PIN_USI_SDA);			// Enable SDA as output.

	USIDR	 =	0xFF;						// Preload data register with "released level" data.
	USICR	 =	(0<<USISIE)|(0<<USIOIE)|					// Disable Interrupts.
				(1<<USIWM1)|(0<<USIWM0)|					// Set USI in Two-wire mode.
				(1<<USICS1)|(0<<USICS0)|(1<<USICLK)|		// Software stobe as counter clock source
				(0<<USITC);
	USISR	=	(1<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|		// Clear flags,
				(0x0<<USICNT0);										// and reset counter.
}

uint8_t usitwim_data_write(uint8_t device_addr, uint8_t *data_buffer, uint8_t data_size)
{
	data_buffer[0] = (device_addr << TWI_ADR_BITS) & ~0x01;
	uint8_t result_code = USI_TWI_Start_Read_Write(data_buffer, data_size);
	// /*DEBUGGING*/ owowod_print_char(data_buffer[0]);
	// /*DEBUGGING*/ owowod_print_char(data_buffer[1]);
	if (result_code != USITWIM_RESULT_SUCCESS) {
//		/*DEBUGGING*/ DEBUGGING_ERROR(result_code, "usitwim_data_write");
	} else {
		// /*DEBUGGING*/ owowod_print_string("(w:ok)");
	}
	return result_code;
}

uint8_t usitwim_data_read(uint8_t device_addr, uint8_t *data_buffer, uint8_t data_size)
{
	data_buffer[0] = (device_addr << TWI_ADR_BITS) | 0x01;
	uint8_t result_code = USI_TWI_Start_Read_Write(data_buffer, data_size);
	// /*DEBUGGING*/ owowod_print_char(data_buffer[0]);
	// /*DEBUGGING*/ owowod_print_char(data_buffer[1]);
	if (result_code != USITWIM_RESULT_SUCCESS) {
//		/*DEBUGGING*/ DEBUGGING_ERROR(result_code, "usitwim_data_read");
	} else {
		// /*DEBUGGING*/ owowod_print_string("(r:ok)");
	}
	return result_code;
}

uint8_t usitwim_mem_read(uint8_t device_addr, uint8_t *data_buffer, uint8_t data_size)
{
	data_buffer[0] = (device_addr << TWI_ADR_BITS) & ~0x01;
	uint8_t result_code = USI_TWI_Start_Random_Read(data_buffer, data_size);
	if (result_code != USITWIM_RESULT_SUCCESS) {
//		/*DEBUGGING*/ DEBUGGING_ERROR(result_code, "usitwim_mem_read");
	} else {
		// /*DEBUGGING*/ owowod_print_string("(m:ok)");
	}
	return result_code;
}

// ----------------------------------------------------------------------------

union USI_TWI_state {
	uint8_t all_bits;
	struct {
		uint8_t addressMode			: 1;
		uint8_t masterWriteDataMode : 1;
		uint8_t memReadMode			: 1;
		uint8_t unused				: 5;
	};
} USI_TWI_state;

/*---------------------------------------------------------------
 USI Random (memory) Read function. This function sets up for call
 to USI_TWI_Start_Transceiver_With_Data which does the work.
 Doesn't matter if read/write bit is set or cleared, it'll be set
 correctly in this function.

 The message_size is passed to USI_TWI_Start_Transceiver_With_Data.

 Success or error code is returned. Error codes are defined in
 the header file.
---------------------------------------------------------------*/
uint8_t USI_TWI_Start_Random_Read(uint8_t *message, uint8_t message_size)
{
	*(message) &= ~(1<<TWI_READ_BIT);	// clear the read bit if it's set
	USI_TWI_state.all_bits = 0;	// Clears all mode bits also
	USI_TWI_state.memReadMode = 1;
	return (USI_TWI_Start_Transceiver_With_Data(message, message_size));	// return status code
}

/*---------------------------------------------------------------
 USI Normal Read / Write Function
 Transmit and receive function. LSB of first byte in buffer
 indicates if a read or write cycles is performed. If set a read
 operation is performed.

 Function generates (Repeated) Start Condition, sends address and
 R/W, Reads/Writes Data, and verifies/sends ACK.

 Success or error code is returned. Error codes are defined in
 the header file.
---------------------------------------------------------------*/
uint8_t USI_TWI_Start_Read_Write( uint8_t *message, uint8_t message_size)
{
	USI_TWI_state.all_bits = 0;	// Clears all mode bits also
	return (USI_TWI_Start_Transceiver_With_Data(message, message_size));	// return status code
}

// This defines the following:
// - Prepare register value to: Clear flags, and
// - set USI to shift 8 bits i.e. count 16 clock edges.
#define USITWIM_USISR_MODE_8BIT \
	(1<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC) | \
	(0x0<<USICNT0)

// This defines the following:
// - Prepare register value to: Clear flags, and
// - set USI to shift 1 bit i.e. count 2 clock edges.
#define USITWIM_USISR_MODE_1BIT \
	(1<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC) | \
	(0xE<<USICNT0)

/*---------------------------------------------------------------
 USI Transmit and receive function. LSB of first byte in buffer
 indicates if a read or write cycles is performed. If set a read
 operation is performed.

 Function generates (Repeated) Start Condition, sends address and
 R/W, Reads/Writes Data, and verifies/sends ACK.

 This function also handles Random Read function if the memReadMode
 bit is set. In that case, the function will:
 The address in memory will be the second
 byte and is written *without* sending a STOP.
 Then the Read bit is set (lsb of first byte), the byte count is
 adjusted (if needed), and the function function starts over by sending
 the slave address again and reading the data.

 Success or error code is returned. Error codes are defined in
 the header file.
---------------------------------------------------------------*/
uint8_t USI_TWI_Start_Transceiver_With_Data( uint8_t *message, uint8_t message_size)
{
	uint8_t result_code;
	uint8_t *saved_message;
	uint8_t saved_message_size;

	// This clear must be done before calling this function so that memReadMode can be specified.
	// USI_TWI_state.all_bits = 0;				// Clears all mode bits also
	USI_TWI_state.addressMode = 1;			// Always 1 for first byte

#ifdef USITWIM_PARAM_VERIFY
	// Test if address is outside SRAM space
	if (message > (uint8_t*)RAMEND) return USI_TWI_DATA_OUT_OF_BOUND;	// return status code
	// Test if the transmission buffer is empty
	if (message_size <= 1) return USI_TWI_NO_DATA;	// return status code
#endif

// Test if any unexpected conditions have arrived prior to this execution.
#ifdef USITWIM_NOISE_TESTING
	if (USISR & (1<<USISIF)) return USI_TWI_UE_START_CON;	// return status code
	if (USISR & (1<<USIPF)) return USI_TWI_UE_STOP_CON;	// return status code
	if (USISR & (1<<USIDC)) return USI_TWI_UE_DATA_COL;	// return status code
#endif

// The LSB in the address byte determines if is a masterRead or masterWrite operation.
	if (!(*message & (1<<TWI_READ_BIT))) USI_TWI_state.masterWriteDataMode = 1;

//	if (USI_TWI_state.memReadMode)
//	{
		saved_message = message;
		saved_message_size = message_size;
//	}

	result_code = USI_TWI_Master_Start();
	if (result_code != USITWIM_RESULT_SUCCESS) return result_code;
	// ERROR: Send a START condition on the TWI bus.
	// NOTE: Return whatever the called function returns.

/*Write address and Read/Write data */
  do
  {
	/* If masterWrite cycle (or initial address transmission)*/
	if (USI_TWI_state.addressMode || USI_TWI_state.masterWriteDataMode)
	{
	  /* Write a byte */
	  PORT_USI &= ~(1<<PIN_USI_SCL);					// Pull SCL LOW.
	  USIDR		= *(message++);							// Setup data.
	  USI_TWI_Master_Transfer(USITWIM_USISR_MODE_8BIT);	// Send 8 bits on bus.

		/* Clock and verify (N)ACK from slave */
		DDR_USI	 &= ~(1<<PIN_USI_SDA);					// Enable SDA as input.
		if (USI_TWI_Master_Transfer(USITWIM_USISR_MODE_1BIT) & (1 << TWI_NACK_BIT))
		{
			if (USI_TWI_state.addressMode)
				return USI_TWI_NO_ACK_ON_ADDRESS;		// return status code
			else
				return USI_TWI_NO_ACK_ON_DATA;			// return status code
		}

	  if ((!USI_TWI_state.addressMode) && USI_TWI_state.memReadMode)
	  // means memory start address has been written
	  {
		message = saved_message;					// start at slave address again
		*(message) |= (1 << TWI_READ_BIT);	// set the Read Bit on Slave address
		USI_TWI_state.all_bits = 0;
		USI_TWI_state.addressMode = 1;	// Now set up for the Read cycle
		message_size = saved_message_size;				// Set byte count correctly
		// Note that the length should be Slave adrs byte + # bytes to read + 1 (gets decremented below)

		result_code = USI_TWI_Master_Start();
		if (result_code != USITWIM_RESULT_SUCCESS) return result_code;
		// ERROR: Send a START condition on the TWI bus.
		// NOTE: Return whatever the called function returns.
		// NOTE: Original code returns USI_TWI_BAD_MEM_READ, but maybe it's wrong code.
	  }
	  else
	  {
		USI_TWI_state.addressMode = 0;			  // Only perform address transmission once.
	  }
	}
	/* Else masterRead cycle*/
	else
	{
	  /* Read a data byte */
	  DDR_USI	&= ~(1<<PIN_USI_SDA);				// Enable SDA as input.
	  *(message++)	= USI_TWI_Master_Transfer(USITWIM_USISR_MODE_8BIT);

	  /* Prepare to generate ACK (or NACK in case of End Of Transmission) */
	  if(message_size == 1)				// If transmission of last byte was performed.
	  {
		USIDR = 0xFF;					// Load NACK to confirm End Of Transmission.
	  }
	  else
	  {
		USIDR = 0x00;					// Load ACK. Set data register bit 7 (output for SDA) low.
	  }
	  USI_TWI_Master_Transfer(USITWIM_USISR_MODE_1BIT);   // Generate ACK/NACK.
	}
  } while (--message_size);								// Until all data sent/received.

	return USI_TWI_Master_Stop();
}

// Prepare clocking.
// This defines the following:
// - Interrupts disabled
// - Set USI in Two-wire mode
// - Software clock strobe as source
// - Toggle Clock Port
#define USITWIM_USICR_MODE \
	(0<<USISIE)|(0<<USIOIE)| \
	(1<<USIWM1)|(0<<USIWM0)| \
	(1<<USICS1)|(0<<USICS0)|(1<<USICLK)| \
	(1<<USITC)							

/*---------------------------------------------------------------
 Core function for shifting data in and out from the USI.
 Data to be sent has to be placed into the USIDR prior to calling
 this function. Data read, will be returned from the function.
---------------------------------------------------------------*/
uint8_t USI_TWI_Master_Transfer(uint8_t mode)
{
	USISR = mode;								// Set USISR according to mode.
	do {
		_delay_us(T2_TWI);
		USICR = USITWIM_USICR_MODE;				// Generate positive SCL edge.
		while (!(PIN_USI & (1<<PIN_USI_SCL)));	// Wait for SCL to go high.
		_delay_us(T4_TWI);
		USICR = USITWIM_USICR_MODE;				// Generate negative SCL edge.
	} while (!(USISR & (1<<USIOIF)));			// Check for transfer complete.
	_delay_us(T2_TWI);
	uint8_t data = USIDR;						// Read out data.
	USIDR = 0xFF;								// Release SDA.
	DDR_USI |= (1 << PIN_USI_SDA);				// Enable SDA as output.
	return data;								// Return the data from the USIDR
}
/*---------------------------------------------------------------
 Function for generating a TWI Start Condition.
---------------------------------------------------------------*/
uint8_t USI_TWI_Master_Start(void)
{
	/* Release SCL to ensure that (repeated) Start can be performed */
	PORT_USI |= (1<<PIN_USI_SCL);				// Release SCL.
	while (!(PORT_USI & (1 << PIN_USI_SCL)));	// Verify that SCL becomes high.
	_delay_us(T2_TWI);

	/* Generate Start Condition */
	PORT_USI &= ~(1<<PIN_USI_SDA);				// Force SDA LOW.
	_delay_us(T4_TWI);
	PORT_USI &= ~(1<<PIN_USI_SCL);				// Pull SCL LOW.
	PORT_USI |= (1<<PIN_USI_SDA);				// Release SDA.

#ifdef USITWIM_SIGNAL_VERIFY
	if (!(USISR & (1<<USISIF)) )
		return USI_TWI_MISSING_START_CON;		// return status code
#endif

	return USITWIM_RESULT_SUCCESS;				// return status code
}
/*---------------------------------------------------------------
 Function for generating a TWI Stop Condition. Used to release
 the TWI bus.
---------------------------------------------------------------*/
uint8_t USI_TWI_Master_Stop(void)
{
	PORT_USI &= ~(1<<PIN_USI_SDA);				// Pull SDA low.
	PORT_USI |= (1<<PIN_USI_SCL);				// Release SCL.
	while (!(PIN_USI & (1 << PIN_USI_SCL)));		// Wait for SCL to go high.
	_delay_us(T4_TWI);
	PORT_USI |= (1<<PIN_USI_SDA);				// Release SDA.
	_delay_us(T2_TWI);

#ifdef USITWIM_SIGNAL_VERIFY
	if (!(USISR & (1<<USIPF)))
		return USI_TWI_MISSING_STOP_CON;		// return status code
#endif

	return USITWIM_RESULT_SUCCESS;				// return status code
}

// ============================================================================
