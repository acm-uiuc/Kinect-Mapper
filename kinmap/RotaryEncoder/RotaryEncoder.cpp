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
#include "RotaryEncoder.h"

void* encoder;

RotaryEncoder::RotaryEncoder(int pinA, int pinB, int pulsesPerRev)
{
	_pinA = pinA;
	_pinB = pinB;
	_pulsesPerRev = pulsesPerRev;
	
	pinMode(_pinA, INPUT_PULLUP); // enable pullup resistor
	pinMode(_pinB, INPUT_PULLUP); // enable pullup resistor
	
	_Aset = (digitalRead(_pinA) == HIGH);
	_Bset = (digitalRead(_pinB) == HIGH);
	
	encoder = (void*) this;
	
	attachInterrupt(_pinA, RotaryEncoder::_interruptHandlerA_wrapper, CHANGE);
	attachInterrupt(_pinB, RotaryEncoder::_interruptHandlerB_wrapper, CHANGE);
}

void RotaryEncoder::_interruptHandlerA_wrapper()
{
	RotaryEncoder* mySelf = (RotaryEncoder*) encoder;
	mySelf->_interruptHandlerA();
}

void RotaryEncoder::_interruptHandlerB_wrapper()
{
	RotaryEncoder* mySelf = (RotaryEncoder*) encoder;
	mySelf->_interruptHandlerB();
}

void RotaryEncoder::reset()
{
	_pulses = 0;
}


int RotaryEncoder::getPulses()
{
	return _pulses;
}


int RotaryEncoder::getRevolutions()
{
	
	return (float) _pulses/_pulsesPerRev;
}


void RotaryEncoder::_interruptHandlerA()
{
	_Aset = digitalRead(_pinA) == HIGH;
	_pulses = (_Aset != _Bset) ? _pulses + 1 : _pulses - 1;
		// Adjust count + if A leads B
}


void RotaryEncoder::_interruptHandlerB()
{
	_Bset = digitalRead(_pinB) == HIGH;
	_pulses = (_Aset == _Bset) ? _pulses + 1 : _pulses - 1;
		// Adjust count + if A follows B
}
