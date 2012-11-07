#include <Servo.h>

#define SERVO_LEFT_PIN 9
#define SERVO_RIGHT_PIN 11
#define LEFT_WHEEL 0
#define RIGHT_WHEEL 1

#define SERVO_FORWARD 180
#define SERVO_STOP 90
#define SERVO_BACKWARDS 0

// magic number 0.037 converts clicks to inches
#define LENGTH 18.25
#define SCALING_FACTOR 0.002027397

Servo servo_left;
Servo servo_right;

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

void setup()
{
    Serial.begin(9600);

    servo_left.attach(SERVO_LEFT_PIN);
    servo_left.write(SERVO_STOP);
    servo_right.attach(SERVO_RIGHT_PIN);
    servo_right.write(SERVO_STOP);

    attachInterrupt(RIGHT_WHEEL, rdist_count, RISING);
    attachInterrupt(LEFT_WHEEL,  ldist_count, RISING);
}


void ldist_count() { ldc++; }
void rdist_count() { rdc++; }

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
