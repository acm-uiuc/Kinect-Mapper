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

#include "Arduino.h"
#include "SimpleRotaryEncoder.h"

void* encoder;

SimpleRotaryEncoder::SimpleRotaryEncoder(int pin, int pulsesPerRev)
{
	_pin = pin;
	_pulsesPerRev = pulsesPerRev;
	
	pinMode(_pin, INPUT_PULLUP); // enable pullup resistor
	
	encoder = (void*) this;
	
	attachInterrupt(_pin, SimpleRotaryEncoder::_interruptHandler_wrapper, CHANGE);
}

void SimpleRotaryEncoder::_interruptHandler_wrapper()
{
	SimpleRotaryEncoder* mySelf = (SimpleRotaryEncoder*) encoder;
	mySelf->_interruptHandler();
}


void SimpleRotaryEncoder::reset()
{
	_pulses = 0;
}


int SimpleRotaryEncoder::getPulses()
{
	return _pulses;
}


int SimpleRotaryEncoder::getRevolutions()
{	
	return (float) _pulses/_pulsesPerRev;
}


void SimpleRotaryEncoder::_interruptHandler()
{
	_pulses++;
}