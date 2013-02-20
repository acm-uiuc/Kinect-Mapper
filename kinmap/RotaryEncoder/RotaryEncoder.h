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

#ifndef RotaryEncoder_h
#define RotaryEncoder_h

#include "Arduino.h"

class RotaryEncoder
{
	public:
		RotaryEncoder(int pinA, int pinB, int pulsesPerRev);
		void reset();
		int getPulses();
		int getRevolutions();
		
		static void _interruptHandlerA_wrapper();
		static void _interruptHandlerB_wrapper();
	
	private:
		void _interruptHandlerA();
		void _interruptHandlerB();
		
		int _pinA;
		int _pinB;
		int _pulsesPerRev;
		
		volatile boolean _Aset;
		volatile boolean _Bset;
		volatile int _pulses;
};

#endif