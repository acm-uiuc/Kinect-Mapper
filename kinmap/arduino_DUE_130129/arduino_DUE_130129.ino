// @reference http://arduino.cc/en/Guide/ArduinoDue for DUE stuff
// @reference http://bit.ly/VoBkia for X2 encoding
// @reference http://mbed.org/cookbook/QEI for QEI Library
// @reference http://mbed.org/users/aberk/code/RS-EDP-RDS-Rover/ for rover library

#include <QEI.h>

#define SERVO_LEFT_PIN 9
#define SERVO_RIGHT_PIN 11

#define LEFT_ENCODER_A 2
#define LEFT_ENCODER_B 3
#define RIGHT_ENCODER_A 4
#define RIGHT_ENCODER_B 5

#define SERVO_FORWARD 180
#define DEFAULT_SERVO_STOP 90
#define SERVO_BACKWARDS 0

// magic number 0.037 converts clicks to inches
#define LENGTH 18.25
#define SCALING_FACTOR 0.002027397
#define SAMPLE_TIME 50 // sample time for geting velocity

Servo servo_left;
Servo servo_right;

// Calibraiton Stuff
volatile int myLeftServoStop;
volatile int myRightServoStop;

// Calculation Stuff
volatile long int ldc = 0; // number of left wheel turns
volatile long int rdc = 0; // number of right wheel turns
long previousVelocityTime = 0; // time at which velocity was last calculated
long timeDifference = 0;
long int ldcprev = 0;
long int rdcprev = 0; // previous count numbers for turns

float Kp = 5.0, Ki = 25.0; // control gains
float velL = 0.0, velR = 0.0;  // measured velocity of each wheel
float errorL = 0.0, errorR = 0.0; // error for controls
float errorLprev = 0.0, errorRprev = 0.0; // error for controls
float uL = 0.0, uR = 0.0; // control inputs for each wheel
float integralL = 0.0, integralR = 0.0; // integral for controls of velocity
float KpTurn = 3.0; // control gain from the coupling and turning
float errorSteering = 0.0; // error input for the coupling
float heading = 0.0; // robot heading

float params[3] = {0.0, 0.0, 0.0}; // {velL, velR, turn}
int signL = 1, signR = 1;
int vl, vr, turn;

// Rotary Encoder Stuff
int _pinA = LEFT_ENCODER_A;
int _pinB = LEFT_ENCODER_B;
int _pinC = RIGHT_ENCODER_A;
int _pinD = RIGHT_ENCODER_B;
volatile boolean _Aset;
volatile boolean _Bset;
volatile boolean _Cset;
volatile boolean _Dset;
volatile long int _Apulses;
volatile long int _Cpulses;

void setup()
{ 
    Serial.begin(9600);
	
	rotaryEncoderSetup();
	servoSetup();

}


/** ---------- Begin Rotary Encoder Stuff ----------*/
// @author Alex Burck

void rotaryEncoderSetup()
{
  pinMode(_pinA, INPUT_PULLUP); // enable pullup resistor
  pinMode(_pinB, INPUT_PULLUP);
  pinMode(_pinC, INPUT_PULLUP);
  pinMode(_pinD, INPUT_PULLUP);
  
  _Apulses = 0;
  _Bpulses = 0;
  
  _Aset = (digitalRead(_pinA) == HIGH);
  _Bset = (digitalRead(_pinB) == HIGH);
  _Cset = (digitalRead(_pinC) == HIGH);
  _Dset = (digitalRead(_pinD) == HIGH);
  
  attachInterrupt(_pinA, _interruptHandlerA, CHANGE);
  attachInterrupt(_pinB, _interruptHandlerB, CHANGE);
  attachInterrupt(_pinC, _interruptHandlerC, CHANGE);
  attachInterrupt(_pinD, _interruptHandlerD, CHANGE);
}

int leftEncoderSpeed () { // returns a speed in pulses/second, takes 50 milliseconds to complete
	int startTime = millis();
	int currentTime;
	long int startLeftCount = _Apulses;
	
	while ( (currentTime = millis()) < (startTime + SAMPLE_TIME) ) { } // waits until at least SAMPLE_TIME has gone by
	
	int timeDifference = currentTime - startTime;
	long int currentLeftCount = _Apulses;
	
    int speedLeft = ((float) (currentLeftCount - startLeftCount)) / timeDifference;
    return speedLeft;
}

int rightEncoderSpeed () { // returns a speed in pulses/second, takes 50 milliseconds to complete
	int startTime = millis();
	int currentTime;
	long int startRightCount = _Cpulses;
	
	while ( (currentTime = millis()) < (startTime + SAMPLE_TIME) ) { } // waits until at least SAMPLE_TIME has gone by
	
	int timeDifference = currentTime - startTime;
	long int currentRightCount = _Cpulses;
	
	int speedRight = ((float) (currentRightCount - startRighttCount)) / timeDifference;
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
  _Dpulses = (_Cset == _Dset) ? _Cpulses + 1 : _Cpulses - 1;
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
	servoCalibrateLeft();
	servoCalibrateRight();
	
	servo_left.write(myLeftServoStop);
    servo_right.write(myRightServoStop);
}

void servoCalibrateLeft() {
	servo_left.write(DEFAULT_SERVO_STOP);
	if (
}

void servoCalibrateRight() {

}

/** ---------- End Servo Stuff ----------*/


void decodeCommand()
{
    // reset the integral term (so that we don't stop for a moment)
    //integralL = 0.0;
    //integralR = 0.0;

    vl   = Serial.read() - 128;
    vr   = Serial.read() - 128;
    turn = Serial.read() - 128; 

    // max speed is 1.0
    params[0] = vl * 0.007874016;
    params[1] = vr * 0.007874016;
    params[2] = turn * 0.002;

    if (vl == 0) {
        signL = 0;
    } else if (vl < 0) {
        signL = -1;
        params[0] *= -1;
    } else {
        signL = 1;
    }

    if (vr == 0) {
        signR = 0;
    } else if (vr < 0) {
        signR = -1;
        params[1] *= -1;
    } else {
        signR = 1;
    }
}


void loop()
{
	if (Serial.available() >= 3) decodeCommand();

    timeDifference = millis() - previousVelocityTime;
    if (timeDifference <= 50) return;
    previousVelocityTime = millis();

    velL = ((float) (ldc - ldcprev)) / timeDifference;
    velR = ((float) (rdc - rdcprev)) / timeDifference;

    errorSteering = KpTurn * (velR - velL + params[2]);

    // Delta_angle = (velL - velR) * (time passed) / (distance between wheels)
    heading += SCALING_FACTOR * ((ldc - ldcprev) - (rdc - rdcprev));

    ldcprev = ldc;
    rdcprev = rdc;


    // left motor control
    errorL = params[0] - velL + errorSteering;
    integralL += (errorL + errorLprev) * .0005 * timeDifference;
    errorLprev = errorL;
    uL = (Kp * errorL) + (integralL * Ki);

    // correct for unbounded integrals
    if (uL > 90) {
        uL = 90;
        integralL *= .99;
    } else if (uL < -90) {
        uL = -90;
        integralL *= .99; 
    }

    // right motor control
    errorR = params[1] - velR - errorSteering;
    integralR += (errorR + errorRprev) * .0005 * timeDifference;
    errorRprev = errorR;
    uR = (Kp * errorR) + (integralR * Ki);


    // correct for unbounded integrals
    if (uR > 90) {
        uR = 90;
        integralR *= .95;
    } else if (uR < -90) {
        uR = -90;
        integralR *= .95; 
    }


    servo_right.write(90 + signR * (int) uR);
    servo_left.write(90 - signL * (int) uL);

/*
    Serial.print(uL);Serial.print("\t");
    Serial.print(ldcprev);Serial.print("\t");

    Serial.print(uR);Serial.print("\t");
    Serial.print(rdcprev);Serial.print("\t");
*/

    Serial.print(velL);Serial.print(" ");
    Serial.print(velR);Serial.print(" ");
    Serial.println(heading);
}
