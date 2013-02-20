/*
	RotaryEncoder.h - Arduino library for recording data from a
		rotary encoder with 2 sensors per encoder.
	
	This library uses X2 encoding referenced here:
		http://mbed.org/cookbook/QEI
		http://bit.ly/VoBkia
		
	Furthermore, general rotary encoder concepts are listed here:
		http://playground.arduino.cc/Main/RotaryEncoders
			
	@author Alex Burck
*/

#ifndef SimpleRotaryEncoder_h
#define SimpleRotaryEncoder_h

#include "Arduino.h"

class SimpleRotaryEncoder
{
	public:
		SimpleRotaryEncoder(int pin, int pulsesPerRev);
		void reset();
		int getPulses();
		int getRevolutions();
		
		static void _interruptHandler_wrapper();
	
	private:
		void _interruptHandler();
		
		int _pin;
		int _pulsesPerRev;

		volatile int _pulses;
};

#endif