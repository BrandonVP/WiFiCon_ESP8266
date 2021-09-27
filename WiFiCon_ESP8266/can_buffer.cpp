/*
 Name:		buffer.c
 Created:	9/3/2021 9:36:16 PM
 Author:	Brandon Van Pelt
*/

#include "can_buffer.h"

#define CAN_BUFFER_C_

#define DEBUG_PUSH
// Copies provided structure into the buffer
bool can_buffer::push(struct CAN_Message addFrame)
{
	// Copy message
	rxBuffer[bufferInPtr].id = addFrame.id;
	for(uint8_t i = 0; i < 8; i++)
	{
		rxBuffer[bufferInPtr].data[i] = addFrame.data[i];
	}

	bufferInPtr++;
#if defined DEBUG_PUSH
	Serial.println("push");
	Serial.print("New InPtr Position: ");
	Serial.println(bufferInPtr);
#endif
	// End of circular buffer
	if (bufferInPtr == (BUFFER_SIZE - 1))
	{
		bufferInPtr = 0;
	}
	// Overflow case
	if (bufferInPtr == bufferOutPtr)
	{
		bufferOutPtr++;
		if (bufferOutPtr == (BUFFER_SIZE - 1))
		{
			bufferOutPtr = 0;
		}
#if defined DEBUG_PUSH
		Serial.println("push");
		Serial.print("New OutPtr Position: ");
		Serial.println(bufferOutPtr);
#endif
		// TODO: Let user know an overwrite occurred
		return false;
	}
	return true;
}

#define DEBUG_POP
// Assign next structure in buffer to provided structure
void can_buffer::pop(struct CAN_Message *bufOut)
{
	// Copy message
	//Serial.print("pop ID: ");
	//Serial.println(rxBuffer[bufferOutPtr].id, 16);
	bufOut->id = rxBuffer[bufferOutPtr].id;
	for(uint8_t i = 0; i < 8; i++)
	{
		bufOut->data[i] = rxBuffer[bufferOutPtr].data[i];
	}

	// Check if empty
	if (bufferOutPtr != bufferInPtr)
	{
		bufferOutPtr++;
#if defined DEBUG_PUSH
		Serial.println("pop");
		Serial.print("New OutPtr Position: ");
		Serial.println(bufferOutPtr);
#endif
		// End of circular buffer
		if (bufferOutPtr > BUFFER_SIZE - 1)
		{
			bufferOutPtr = 0;
		}
	}
}

// Calculates current structures in buffer by subtracting points then anding with max buffer size value
uint8_t can_buffer::stack_size(void)
{
	uint8_t size = (bufferInPtr - bufferOutPtr) & (BUFFER_SIZE - 1);
	return size;
}

// Check the next structure int he buffer without incrementing
void can_buffer::peek(struct CAN_Message* peek)
{
	peek->id = rxBuffer[bufferOutPtr].id;
	for(uint8_t i = 0; i < 8; i++)
	{
		peek->data[i] = rxBuffer[bufferInPtr].data[i];
	}
}

// Reset both points back to zero
void can_buffer::clear_buffer()
{
	bufferOutPtr = 0;
	bufferInPtr = 0;
}
