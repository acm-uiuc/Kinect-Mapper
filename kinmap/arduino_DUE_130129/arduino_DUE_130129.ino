#include <Servo.h>

// @reference http://arduino.cc/en/Guide/ArduinoDue for DUE stuff
// @reference http://bit.ly/VoBkia for X2 encoding
// @reference http://mbed.org/cookbook/QEI for QEI Library
// @reference http://mbed.org/users/aberk/code/RS-EDP-RDS-Rover/ for rover library

#define SERVO_LEFT_PIN 9
#define SERVO_RIGHT_PIN 11

#define LEFT_ENCODER_A 2
#define LEFT_ENCODER_B 3
#define RIGHT_ENCODER_A 5
#define RIGHT_ENCODER_B 6

#define SERVO_FORWARD 180
#define DEFAULT_SERVO_STOP 90
#define SERVO_BACKWARDS 0


#define SAMPLE_TIME 20 // sample time for geting speed
#define CALIBRATION_ACCURACY 2 // calibration is +/- 5 pulses/millisecond
#define CALIBRATION_DELAY 10 // delay between writing to servo

Servo servo_left;
Servo servo_right;

// Calibraiton Stuff
volatile int myLeftServoStop;
volatile int myRightServoStop;


// Rotary Encoder Stuff
int _pinA = LEFT_ENCODER_A;
int _pinB = LEFT_ENCODER_B;
int _pinC = RIGHT_ENCODER_A;
int _pinD = RIGHT_ENCODER_B;
volatile boolean _Aset;
volatile boolean _Bset;
volatile boolean _Cset;
volatile boolean _Dset;
volatile long _Apulses;
volatile long _Cpulses;


/** ---------- Begin Rotary Encoder Stuff ----------*/
// @author Alex Burck

void rotaryEncoderSetup()
{
  pinMode(_pinA, INPUT_PULLUP); // enable pullup resistor
  pinMode(_pinB, INPUT_PULLUP);
  pinMode(_pinC, INPUT_PULLUP);
  pinMode(_pinD, INPUT_PULLUP);
  
  _Apulses = 0;
  _Cpulses = 0;
  
  _Aset = (digitalRead(_pinA) == HIGH);
  _Bset = (digitalRead(_pinB) == HIGH);
  _Cset = (digitalRead(_pinC) == HIGH);
  _Dset = (digitalRead(_pinD) == HIGH);
  
  attachInterrupt(_pinA, _interruptHandlerA, CHANGE);
  attachInterrupt(_pinB, _interruptHandlerB, CHANGE);
  attachInterrupt(_pinC, _interruptHandlerC, CHANGE);
  attachInterrupt(_pinD, _interruptHandlerD, CHANGE);
}

int leftEncoderSpeed() { // returns a speed in pulses/millisecond, takes 50 milliseconds to complete
	int startTime = millis();
	int currentTime;
	long startLeftCount = _Apulses;
	
	while ( (currentTime = millis()) < (startTime + SAMPLE_TIME) ) { } // waits until at least SAMPLE_TIME has gone by
	
	int timeDifference = currentTime - startTime;
	long currentLeftCount = _Apulses;
	
	int diffCount = currentLeftCount - startLeftCount;
	diffCount = abs(diffCount);
    int speedLeft = ((float) diffCount) / timeDifference;
    return speedLeft;
}

int rightEncoderSpeed() { // returns a speed in pulses/millisecond, takes 50 milliseconds to complete
	int startTime = millis();
	int currentTime;
	long startRightCount = _Cpulses;
	
	while ( (currentTime = millis()) < (startTime + SAMPLE_TIME) ) { } // waits until at least SAMPLE_TIME has gone by
	
	int timeDifference = currentTime - startTime;
	long currentRightCount = _Cpulses;
	
	int diffCount = currentRightCount - startRightCount;
	diffCount = abs(diffCount);
	int speedRight = ((float) diffCount) / timeDifference;
    return speedRight;
}

void _interruptHandlerA()
{
  _Aset = digitalRead(_pinA) == HIGH;
  _Apulses = (_Aset != _Bset) ? _Apulses + 1 : _Apulses - 1;
    // Adjust count + if A leads B
}

void _interruptHandlerB()
{
  _Bset = digitalRead(_pinB) == HIGH;
  _Apulses = (_Aset == _Bset) ? _Apulses + 1 : _Apulses - 1;
    // Adjust count + if A follows B
}

void _interruptHandlerC()
{
  _Cset = digitalRead(_pinC) == HIGH;
  _Cpulses = (_Cset != _Dset) ? _Cpulses + 1 : _Cpulses - 1;
    // Adjust count + if C leads D
}

void _interruptHandlerD()
{
  _Dset = digitalRead(_pinD) == HIGH;
  _Cpulses = (_Cset == _Dset) ? _Cpulses + 1 : _Cpulses - 1;
    // Adjust count + if C follows D
}

/** ---------- End Rotary Encoder Stuff ----------*/

/** ---------- Begin Servo Stuff ----------*/
// @author Alex Burck

void servoSetup () {
    servo_left.attach(SERVO_LEFT_PIN);
    servo_right.attach(SERVO_RIGHT_PIN);
    
    servoCalibrate();
}

void servoCalibrate() {
	servoCalibrateLeft(DEFAULT_SERVO_STOP, leftEncoderSpeed());
	servoCalibrateRight(DEFAULT_SERVO_STOP, rightEncoderSpeed());
	
	servo_left.write(myLeftServoStop);
    servo_right.write(myRightServoStop);
}

void servoCalibrateLeft(int servoSpeed, int prevEncoderSpeed) {
	servo_left.write(servoSpeed);
	int encoderSpeed = leftEncoderSpeed();
	if (encoderSpeed <= CALIBRATION_ACCURACY) return;

	if (encoderSpeed > prevEncoderSpeed) {
		servoSpeed = servoSpeed + 1;
		servo_left.write(servoSpeed);
	}
	else { // if (encoderSpeed <= prevEncoderSpeed)
		servoSpeed = servoSpeed - 1;
		servo_left.write(servoSpeed);
	}

	delay(CALIBRATION_DELAY); // 10 ms
}

void servoCalibrateRight(int servoSpeed, int prevEncoderSpeed) {
	servo_right.write(servoSpeed);
	int encoderSpeed = rightEncoderSpeed();
	if (encoderSpeed <= CALIBRATION_ACCURACY) return;

	if (encoderSpeed > prevEncoderSpeed) {
		servoSpeed = servoSpeed + 1;
		servo_right.write(servoSpeed);
	}
	else { // if (encoderSpeed <= prevEncoderSpeed)
		servoSpeed = servoSpeed - 1;
		servo_right.write(servoSpeed);
	}
	delay(CALIBRATION_DELAY); // 10 ms
}

/** ---------- End Servo Stuff ----------*/

void setup()
{ 
    //Serial.begin(9600);
	
	rotaryEncoderSetup();
	servoSetup();
}

void loop()
{
	// Currently only doing setup
}
