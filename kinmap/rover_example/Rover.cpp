/**
 * @author Aaron Berk
 *
 * @section LICENSE
 *
 * Copyright (c) 2010 ARM Limited
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * RS-EDP Rover application class.
 *
 * Demonstrates four action states: moving {forward, backward},
 *                                  rotating {clockwise, counter-clockwise}.
 *
 * Logs heading and left and right motor position and velocity data.
 *
 * Performs PID velocity control on the motors.
 *
 * ---------------
 *  CONFIGURATION
 * ---------------
 *
 * The set up assumes the H-bridge being used has pins for:
 *
 * - PWM input
 * - Brake
 * - Direction
 *
 * The constructor arguments will need to be changed if a different type of
 * H-bridge is used.
 *
 * The PID controllers are configured using the #defines below.
 *
 * The set up also assumes two quadrature encoders are used, each using the
 * default X2 encoding.
 *
 * The constructor arguments will need to be changed if a different number
 * of encoders are used or if a different encoding is required.
 */

/**
 * Includes
 */
#include "Rover.h"

Rover::Rover(PinName leftMotorPwm,
             PinName leftMotorBrake,
             PinName leftMotorDirection,
             PinName rightMotorPwm,
             PinName rightMotorBrake,
             PinName rightMotorDirection,
             PinName leftQeiChannelA,
             PinName leftQeiChannelB,
             PinName leftQeiIndex,
             int leftPulsesPerRev,
             PinName rightQeiChannelA,
             PinName rightQeiChannelB,
             PinName rightQeiIndex,
             int rightPulsesPerRev) :
        leftMotors(),
        rightMotors(),
        leftQei(leftQeiChannelA,
                leftQeiChannelB,
                leftQeiIndex,
                leftPulsesPerRev),
        rightQei(rightQeiChannelA,
                 rightQeiChannelB,
                 rightQeiIndex,
                 rightPulsesPerRev),
        leftController(Kc, Ti, Td, PID_RATE),
        rightController(Kc, Ti, Td, PID_RATE),
        stateTicker(),
        logTicker(),
        imu(IMU_RATE_,
            GYRO_MEAS_ERROR,
            ACCELEROMETER_RATE,
            GYROSCOPE_RATE) {

    //---------------------------------
    // Left motors and PID controller.
    //---------------------------------

    //Motors.
    leftMotors.setPwmPin(leftMotorPwm);
    leftMotors.setBrakePin(leftMotorBrake);
    leftMotors.setDirectionPin(leftMotorDirection);
    leftMotors.initialize();
    leftMotors.setDirection(BACKWARD);
    leftMotors.setBrake(BRAKE_OFF);

    //PID.
    leftController.setInputLimits(PID_IN_MIN, PID_IN_MAX);
    leftController.setOutputLimits(PID_OUT_MIN, PID_OUT_MAX);
    leftController.setBias(PID_BIAS);
    leftController.setMode(AUTO_MODE);
    leftPulses_     = 0;
    leftPrevPulses_ = 0;
    leftPwmDuty_    = 1.0;
    leftVelocity_   = 0.0;

    //----------------------------------
    // Right motors and PID controller.
    //----------------------------------

    //Motors.
    rightMotors.setPwmPin(rightMotorPwm);
    rightMotors.setBrakePin(rightMotorBrake);
    rightMotors.setDirectionPin(rightMotorDirection);
    rightMotors.initialize();
    rightMotors.setDirection(FORWARD);
    rightMotors.setBrake(BRAKE_OFF);

    //PID.
    rightController.setInputLimits(PID_IN_MIN, PID_IN_MAX);
    rightController.setOutputLimits(PID_OUT_MIN, PID_OUT_MAX);
    rightController.setBias(PID_BIAS);
    rightController.setMode(AUTO_MODE);
    rightPulses_     = 0;
    rightPrevPulses_ = 0;
    rightPwmDuty_    = 1.0;
    rightVelocity_   = 0.0;

    //--------------------
    // Working Variables.
    //--------------------
    positionSetPoint_ = 0.0;
    headingSetPoint_  = 0.0;
    heading_          = 0.0;
    prevHeading_      = 0.0;
    degreesTurned_    = 0.0;
    leftStopFlag_     = 0;
    rightStopFlag_    = 0;
    logIndex          = 0;

    //--------
    // BEGIN!
    //--------
    state_ = STATE_STATIONARY;
    stateTicker.attach(this, &Rover::doState, PID_RATE);
    startLogging();

}

void Rover::move(float distance) {

    //Convert from metres into millimetres.
    distance *= 1000;
    //Work out how many pulses are required to go that many millimetres.
    distance *= PULSES_PER_MM;
    //Make sure we scale the number of pulses according to our encoding method.
    distance /= ENCODING;

    positionSetPoint_ = distance;

    //Moving forward.
    if (distance > 0) {

        enterState(STATE_MOVING_FORWARD);

    }
    //Moving backward.
    else if (distance < 0) {

        enterState(STATE_MOVING_BACKWARD);

    }

    //If distance == 0, then do nothing, i.e. stay stationary.

}

void Rover::turn(int degrees) {

    //Correct the amount to turn based on deviation during last segment.
    headingSetPoint_ = abs(degrees) + (endHeading_ - startHeading_);
    
    //In case the rover tries to [pointlessly] turn >360 degrees.
    if (headingSetPoint_ > 359.8){
    
        headingSetPoint_ -= 359.8;
    
    }

    //Rotating clockwise.
    if (degrees > 0) {

        enterState(STATE_ROTATING_CLOCKWISE);

    }
    //Rotating counter-clockwise.
    else if (degrees < 0) {

        enterState(STATE_ROTATING_COUNTER_CLOCKWISE);

    }

    //If degrees == 0, then do nothing, i.e. stay stationary.

}

Rover::State Rover::getState(void) {

    return state_;

}

void Rover::startLogging(void) {

    logFile = fopen("/local/roverlog.csv", "w");
    fprintf(logFile, "leftPulses, rightPulses, leftVelocity, rightVelocity, heading\n");
    //logTicker.attach(this, &Rover::log, LOG_RATE);

}

void Rover::stopLogging(void) {

    //logFile = fopen("/local/roverlog.csv", "w");
    //fprintf(logFile, "leftPulses, rightPulses, leftVelocity, rightVelocity, heading, degreesTurned\n");
    //fprintf(logFile, "leftVelocity, rightVelocity\n");
    //for(int i = 0; i < 1024; i++){
    //    fprintf(logFile, "%f, %f\n", leftVelocityBuffer[i], rightVelocityBuffer[i]);
    //}
    fclose(logFile);

}

void Rover::log(void) {

    //fprintf(logFile, "%i,%i,%f,%f,%f,%f\n",
    //        leftPulses_, rightPulses_, leftVelocity_, rightVelocity_, imu.getYaw(), degreesTurned_);

}

void Rover::doState(void) {

    switch (state_) {

            //We're not moving so don't do anything!
        case (STATE_STATIONARY):

            break;

        case (STATE_MOVING_FORWARD):

            //If we haven't hit the position set point yet,
            //perform velocity control on the motors.
            if (leftPulses_ < positionSetPoint_) {

                leftPulses_ = leftQei.getPulses();
                leftVelocity_ = (leftPulses_ - leftPrevPulses_) / PID_RATE;
                leftPrevPulses_ = leftPulses_;
                leftController.setProcessValue(leftVelocity_);
                leftPwmDuty_ = leftController.compute();

            } else {
                leftStopFlag_ = 1;
            }

            leftMotors.setPwmDuty(leftPwmDuty_);

            if (rightPulses_ < positionSetPoint_) {

                rightPulses_ = rightQei.getPulses();
                rightVelocity_ = (rightPulses_ - rightPrevPulses_) / PID_RATE;
                rightPrevPulses_ = rightPulses_;
                rightController.setProcessValue(rightVelocity_);
                rightPwmDuty_ = rightController.compute();

            } else {
                rightStopFlag_ = 1;
            }

            rightMotors.setPwmDuty(rightPwmDuty_);

            //We've hit the end position set point.
            if (leftStopFlag_ == 1 && rightStopFlag_ == 1) {
                leftPwmDuty_  = 1.0;
                rightPwmDuty_ = 1.0;
                leftMotors.setPwmDuty(leftPwmDuty_);
                rightMotors.setPwmDuty(rightPwmDuty_);
                endHeading_ = imu.getYaw();
                enterState(STATE_STATIONARY);
            }

            break;

        case (STATE_MOVING_BACKWARD):

            //If we haven't hit the position set point yet,
            //perform velocity control on the motors.
            if (leftPulses_ > positionSetPoint_) {
                leftPulses_ = leftQei.getPulses();
                leftVelocity_ = (leftPulses_ - leftPrevPulses_) / PID_RATE;
                leftPrevPulses_ = leftPulses_;
                leftController.setProcessValue(fabs(leftVelocity_));
                leftPwmDuty_ = leftController.compute();
            } else {
                leftStopFlag_ = 1;
            }

            leftMotors.setPwmDuty(leftPwmDuty_);

            if (rightPulses_ > positionSetPoint_) {
                rightPulses_ = rightQei.getPulses();
                rightVelocity_ = (rightPulses_ - rightPrevPulses_) / PID_RATE;
                rightPrevPulses_ = rightPulses_;
                rightController.setProcessValue(fabs(rightVelocity_));
                rightPwmDuty_ = rightController.compute();

            } else {
                rightStopFlag_ = 1;
            }

            rightMotors.setPwmDuty(rightPwmDuty_);

            //We've hit the end position set point.
            if (leftStopFlag_ == 1.0 && rightStopFlag_ == 1.0) {
                leftPwmDuty_  = 1.0;
                rightPwmDuty_ = 1.0;
                leftMotors.setPwmDuty(leftPwmDuty_);
                rightMotors.setPwmDuty(rightPwmDuty_);
                enterState(STATE_STATIONARY);
            }

            break;

        case (STATE_ROTATING_CLOCKWISE):

            //If we haven't hit the position set point yet,
            //perform velocity control on the motors.
            if (degreesTurned_ < headingSetPoint_) {

                heading_ = fabs(imu.getYaw());
                degreesTurned_ += fabs(heading_ - prevHeading_);
                prevHeading_ = heading_;

                leftPulses_ = leftQei.getPulses();
                leftVelocity_ = (leftPulses_ - leftPrevPulses_) / PID_RATE;
                leftPrevPulses_ = leftPulses_;
                leftController.setProcessValue(leftVelocity_);
                leftPwmDuty_ = leftController.compute();

                rightPulses_ = rightQei.getPulses();
                rightVelocity_ = (rightPulses_ - rightPrevPulses_) / PID_RATE;
                rightPrevPulses_ = rightPulses_;
                rightController.setProcessValue(fabs(rightVelocity_));
                rightPwmDuty_ = rightController.compute();

                leftMotors.setPwmDuty(leftPwmDuty_);
                rightMotors.setPwmDuty(rightPwmDuty_);

            } else {

                leftPwmDuty_  = 1.0;
                rightPwmDuty_ = 1.0;
                leftMotors.setPwmDuty(leftPwmDuty_);
                rightMotors.setPwmDuty(rightPwmDuty_);
                enterState(STATE_STATIONARY);

            }

            break;

        case (STATE_ROTATING_COUNTER_CLOCKWISE):

            //If we haven't hit the position set point yet,
            //perform velocity control on the motors.
            if (degreesTurned_ < headingSetPoint_) {

                heading_ = fabs(imu.getYaw());
                degreesTurned_ += fabs(heading_ - prevHeading_);
                prevHeading_ = heading_;

                leftPulses_ = leftQei.getPulses();
                leftVelocity_ = (leftPulses_ - leftPrevPulses_) / PID_RATE;
                leftPrevPulses_ = leftPulses_;
                leftController.setProcessValue(fabs(leftVelocity_));
                leftPwmDuty_ = leftController.compute();

                rightPulses_ = rightQei.getPulses();
                rightVelocity_ = (rightPulses_ - rightPrevPulses_) / PID_RATE;
                rightPrevPulses_ = rightPulses_;
                rightController.setProcessValue(rightVelocity_);
                rightPwmDuty_ = rightController.compute();

                leftMotors.setPwmDuty(leftPwmDuty_);
                rightMotors.setPwmDuty(rightPwmDuty_);

            } else {

                leftPwmDuty_  = 1.0;
                rightPwmDuty_ = 1.0;
                leftMotors.setPwmDuty(leftPwmDuty_);
                rightMotors.setPwmDuty(rightPwmDuty_);
                enterState(STATE_STATIONARY);

            }

            break;

            //If we've fallen into a black hole, the least we can do is turn
            //the motors off.
        default:

            leftMotors.setPwmDuty(1.0);
            rightMotors.setPwmDuty(1.0);

            break;

    }

    if (logIndex < 1024) {
        headingBuffer[logIndex] = imu.getYaw();
        leftVelocityBuffer[logIndex] = leftVelocity_;
        rightVelocityBuffer[logIndex] = rightVelocity_;
        leftPositionBuffer[logIndex] = leftPulses_;
        rightPositionBuffer[logIndex] = rightPulses_;
        logIndex++;
    }

}

void Rover::enterState(State state) {

    switch (state) {

            //Entering stationary state.
            //1. Turn motors off.
            //2. Reset QEIs.
            //3. Reset PID working variables.
            //4. Reset PIDs.
            //5. Reset set points and working variables.
            //7. Set state variable.
        case (STATE_STATIONARY):

            leftMotors.setPwmDuty(1.0);
            rightMotors.setPwmDuty(1.0);

            leftQei.reset();
            rightQei.reset();

            leftPulses_     = 0;
            leftPrevPulses_ = 0;
            leftPwmDuty_    = 1.0;
            leftVelocity_   = 0.0;

            rightPulses_     = 0;
            rightPrevPulses_ = 0;
            rightPwmDuty_    = 1.0;
            rightVelocity_   = 0.0;

            leftController.setSetPoint(0.0);
            leftController.setProcessValue(0.0);
            rightController.setSetPoint(0.0);
            rightController.setProcessValue(0.0);

            positionSetPoint_ = 0.0;
            headingSetPoint_  = 0.0;
            heading_          = 0.0;
            prevHeading_      = 0.0;
            degreesTurned_    = 0.0;
            leftStopFlag_     = 0;
            rightStopFlag_    = 0;

            for (int i = 0; i < logIndex; i++) {
                fprintf(logFile, "%i, %i, %f, %f, %f\n", leftPositionBuffer[i],
                        rightPositionBuffer[i],
                        leftVelocityBuffer[i],
                        rightVelocityBuffer[i],
                        headingBuffer[i]);
            }

            logIndex = 0;
            
            imu.reset();

            state_ = STATE_STATIONARY;

            break;

            //Entering moving forward state.
            //1. Set correct direction for motors.
            //2. Set velocity set point.
            //3. Set state variable.
        case (STATE_MOVING_FORWARD):

            leftMotors.setDirection(BACKWARD);
            rightMotors.setDirection(FORWARD);

            //Velocity control.
            leftController.setSetPoint(1000);
            rightController.setSetPoint(1000);

            logIndex = 0;
            
            startHeading_ = imu.getYaw();

            state_ = STATE_MOVING_FORWARD;

            break;

            //Entering moving backward state.
            //1. Set correct direction for motors.
            //2. Set velocity set point.
            //3. Set state variable.
        case (STATE_MOVING_BACKWARD):

            leftMotors.setDirection(FORWARD);
            rightMotors.setDirection(BACKWARD);

            //Velocity control.
            leftController.setSetPoint(1000);
            rightController.setSetPoint(1000);

            logIndex = 0;

            state_ = STATE_MOVING_BACKWARD;

            break;

            //Entering rotating clockwise state.
            //1. Set correct direction for motors.
            //2. Set velocity set point.
            //3. Set working variables.
            //4. Set state variable.
        case (STATE_ROTATING_CLOCKWISE):

            leftMotors.setDirection(BACKWARD);
            rightMotors.setDirection(BACKWARD);

            leftController.setSetPoint(500);
            rightController.setSetPoint(500);

            degreesTurned_ = 0.0;
            heading_ = fabs(imu.getYaw());
            prevHeading_ = heading_;

            logIndex = 0;

            state_ = STATE_ROTATING_CLOCKWISE;

            break;

            //Entering rotating clockwise state.
            //1. Set correct direction for motors.
            //2. Set velocity set point.
            //3. Set working variables.
            //4. Set state variable.
        case (STATE_ROTATING_COUNTER_CLOCKWISE):

            leftMotors.setDirection(FORWARD);
            rightMotors.setDirection(FORWARD);

            leftController.setSetPoint(500);
            rightController.setSetPoint(500);

            degreesTurned_ = 0.0;
            heading_ = fabs(imu.getYaw());
            prevHeading_ = heading_;

            logIndex = 0;

            state_ = STATE_ROTATING_COUNTER_CLOCKWISE;

            break;

        default:

            state_ = STATE_STATIONARY;

            break;

    }

}
