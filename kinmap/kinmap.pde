#include <Servo.h>

#define SERVO_LEFT_PIN 11
#define SERVO_RIGHT_PIN 9
#define LEFT_WHEEL 1
#define RIGHT_WHEEL 0

#define SERVO_FORWARD 180
#define SERVO_STOP 90
#define SERVO_BACKWARDS 0

Servo servo_left;
Servo servo_right;

volatile long int ldc = 0; //number of left wheel turns
volatile long int rdc = 0; //number of right wheel turns
long previousVelocityTime = 0; //time at which velocity was last calculated
long timeDifference = 0;
long int ldcprev = 0;
long int rdcprev = 0; //previous count numbers for turns

float Kp = 20.0, Ki = 25.0, Kd = 10.0; //control gains
float velL = 0.0, velR = 0.0;  //velocity of each wheel
float errorL = 0.0, errorR = 0.0; //error for controls
float errorLprev = 0.0, errorRprev = 0.0; //error for controls
float uL = 0.0, uR = 0.0; //control inputs for each wheel
float vRefL = 0.0, vRefR = 0.0; //desired velocity for each wheel
float integralL = 0.0, integralR = 0.0; //integral for controls of velocity
float differentialL = 0.0, differentialR = 0.0; //differential for controls of velocity
float KpTurn = .5; //control gain from the coupling and turning
float turn = 0.0; //turn control input
float errorSteering = 0.0; //error input for the coupling

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


void ldist_count()
{
    ldc++;
}

void rdist_count()
{  
    rdc++;  
}

void loop()
{
    vRefL = .15;
    vRefR = .15;
    turn = -0.025;

    timeDifference = millis() - previousVelocityTime;
    if(timeDifference > 50){ previousVelocityTime = millis();

    velL = ((float) (ldc - ldcprev)) / (float)timeDifference;
    velR = ((float) (rdc - rdcprev)) / (float)timeDifference;

    errorSteering = KpTurn * (velR - velL + turn);


/*
    Serial.print(ldc);Serial.print(" ");Serial.print(ldcprev);Serial.print(" ");Serial.println(ldc-ldcprev);
    Serial.print(rdc);Serial.print(" ");Serial.print(rdcprev);Serial.print(" ");Serial.println(rdc-rdcprev);
    Serial.println(timeDifference);
*/

    ldcprev = ldc;
    rdcprev = rdc;

    // left motor control
    errorL = vRefL - velL + errorSteering;
    integralL += (errorL + errorLprev) * .0005 * timeDifference;
    //differentialL = (errorL - errorLprev) / timeDifference;
    //Serial.println(errorLprev);
    errorLprev = errorL;
    //uL = (Kp * errorL) + (Ki * integralL) + (Kd * differentialL);
    uL = (Kp * errorL) + (integralL * Ki);

    if(uL > 90)
    {
        uL = 90;
        integralL *= .99;
    }
    else if(uL < -90)
    {
        uL = -90;
        integralL *= .99; 
    }

    // right motor control
    errorR = vRefR - velR - errorSteering;
    integralR += (errorR + errorRprev) * .0005 * timeDifference;
    //differentialR = (errorR - errorRprev) / timeDifference;
    //Serial.println(errorRprev);
    errorRprev = errorR;
    //uR = (Kp * errorR) + (Ki * integralR) + (Kd * differentialR);
    uR = (Kp * errorR) + (integralR * Ki);

    if(uR > 90)
    {
        uR = 90;
        integralR *= .95;
    }
    else if(uR < -90)
    {
        uR = -90;
        integralR *= .95; 
    }

//if((millis()%20) == 0){

    //servo_right.write(103);
    //servo_left.write(103);
    servo_right.write(90 + (int) uR);
    servo_left.write(90 - (int) uL);
//}


//if((millis() % 50 )== 0){
    Serial.print(uL);Serial.print(" ");
    Serial.print(ldcprev);Serial.print(" ");
    Serial.print(velL);Serial.print(" ");
    //Serial.print(errorL);Serial.print(" ");
    //Serial.print(integralL);Serial.print(" ");
    //Serial.println(differentialL);

    Serial.print(uR);Serial.print(" ");
    Serial.print(rdcprev);Serial.print(" ");
    Serial.println(velR);Serial.print(" ");
    //Serial.print(errorR);Serial.print(" ");
    //Serial.println(integralR);Serial.print(" ");
    //Serial.println(differentialR);

    //Serial.println(); Serial.println();
//}
    }
}
