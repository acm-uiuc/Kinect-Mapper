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

#ifndef ROVER_H
#define ROVER_H

/**
 * Includes
 */
#include "mbed.h"
#include "RSEDP_AM_MC1.h"
#include "QEI.h"
#include "PID.h"
#include "IMU.h"
#include "SDFileSystem.h"
#include "HMC6352.h"

/**
 * Defines
 */
//---------------------
// Physical attributes
//---------------------
#define PULSES_PER_REV        624
#define WHEEL_DIAMETER        58.928 //mm
#define ROTATION_DISTANCE     220.0  //mm
#define REVS_PER_ROTATION     (ROTATION_DISTANCE / WHEEL_DIAMETER)
#define PULSES_PER_ROTATION   (REVS_PER_ROTATION * PULSES_PER_REV)
#define PULSES_PER_MM         (PULSES_PER_REV / WHEEL_DIAMETER)
#define DISTANCE_PER_PULSE    (WHEEL_DIAMETER / PULSES_PER_REV)
#define ENCODING              2 //Use X2 encoding
#define WHEEL_DISTANCE        (ROTATION_DISTANCE / DISTANCE_PER_PULSE)
//-----
// PID
//-----
#define PID_RATE     0.01
#define PID_BIAS     1.0
#define PID_IN_MIN   0.0
#define PID_IN_MAX   10500.0
#define PID_OUT_MIN  0.0
#define PID_OUT_MAX  1.0
#define Kc          -0.9
#define Ti           0.08
#define Td           0.0
//---------
// Logging
//---------
#define LOG_RATE     0.025
//-----
// IMU
//-----
#define ACCELEROMETER_RATE 0.005
#define GYROSCOPE_RATE     0.005
#define IMU_RATE_          0.025
#define GYRO_MEAS_ERROR    0.3

class Rover {

public:

    typedef enum State {

        STATE_STATIONARY,
        STATE_MOVING_FORWARD,
        STATE_MOVING_BACKWARD,
        STATE_ROTATING_CLOCKWISE,
        STATE_ROTATING_COUNTER_CLOCKWISE

    } State;

    /**
     * Constructor.
     *
     * Creates left and right motor control module objects using the specified
     * pins and initializes them. Sets the direction appropriately so both
     * wheels are going "forward".
     * Creates the left and right wheel quadrature encoder interfaces with the
     * specified pins.
     *
     * @param leftMotorPwm Pin to use for PWM input for the left motors.
     * @param leftMotorBrake Pin to use for brake input for left motors.
     * @param leftMotorDirection Pin to use for direction input for the left
     *                           motors.
     * @param rightMotorPwm Pin to use for PWM input for the right motors.
     * @param rightMotorBrake Pin to use for brake input for right motors.
     * @param rightMotorDirection Pin to use for direction input for the right
     *                            motors.
     * @param leftQeiChannelA Pin to use for channel A on the left wheel
     *                        quadrature encoder.
     * @param leftQeiChannelB Pin to use for channel B on the left wheel
     *                        quadrature encoder.
     * @param leftQeiIndex Pin to use for the index channel on the left wheel
     *                     quadrature encoder.
     * @param leftPulsesPerRev The number of pulses per revolution on the left
     *                         quadrature encoder.
     * @param rightQeiChannelA Pin to use for channel A on the right wheel
     *                         quadrature encoder.
     * @param rightQeiChannelB Pin to use for channel B on the right wheel
     *                         quadrature encoder.
     * @param rightQeiIndex Pin to use for the index channel on the left wheel
     *                      quadrature encoder.
     * @param rightPulsesPerRev The number of pulses per revolution on the
     *                          right quadrature encoder.
     */
    Rover(PinName leftMotorPwm,
          PinName leftMotorBrake,
          PinName leftMotorDirection,
          PinName rightMotorPwm,
          PinName rightMotorBrake,
          PinName rightMotorDirection,
          PinName leftQeiChannelA,
          PinName leftQeiChannelB,
          PinName leftQeiIndex,
          int     leftPulsesPerRev,
          PinName rightQeiChannelA,
          PinName rightQeiChannelB,
          PinName rightQeiIndex,
          int     rightPulsesPerRev);

    /**
     * Move the rover directly forward/backward a certain distance.
     *
     * Distance measured in pulses/stripes of the encoder wheel.
     * +ve distance -> forward
     * -ve distance -> backward
     *
     * @param distance The distance to move in metres.
     */
    void move(float distance);

    /**
     * Turn the rover left or right a certain number of degrees.
     *
     * +ve degrees -> clockwise
     * -ve degrees -> counter-clockwise
     *
     * @param degrees The number of degrees to rotate.
     */
    void turn(int degrees);

    /**
     * Get the current state of the rover.
     *
     * @return The current state of the rover.
     */
    State getState(void);

    /**
     * Start logging position, velocity and heading data.
     */
    void startLogging(void);

    /**
     * Stop logging position, velocity and heading data.
     */
    void stopLogging(void);

private:

    /**
     * Takes appropriate action for the current state of the Rover.
     */
    void doState(void);

    /**
     * Actions to take on entering a new state.
     *
     * @param state The new state to enter.
     */
    void enterState(State state);

    /**
     * Set up the accelerometer for our application.
     */
    void initializeAccelerometer(void);

    /**
     * Calibrate the accelerometer.
     */
    void calibrateAccelerometer(void);

    /**
     * Get the latest accelerometer data.
     */
    void sampleAccelerometer(void);

    /**
     * Log the current position, velocity and heading data.
     */
    void log(void);

    RSEDP_AM_MC1  leftMotors;
    RSEDP_AM_MC1  rightMotors;
    QEI           leftQei;
    QEI           rightQei;
    PID           leftController;
    PID           rightController;
    Ticker        stateTicker;
    Ticker        logTicker;
    IMU           imu;

    FILE*          logFile;

    volatile int   leftStopFlag_;
    volatile int   rightStopFlag_;

    volatile int   positionSetPoint_;
    volatile float headingSetPoint_;
    volatile float degreesTurned_;

    volatile int   leftPulses_;
    volatile int   leftPrevPulses_;
    volatile float leftPwmDuty_;
    volatile float leftVelocity_;

    volatile int   rightPulses_;
    volatile int   rightPrevPulses_;
    volatile float rightPwmDuty_;
    volatile float rightVelocity_;

    volatile float prevHeading_;
    volatile float heading_;

    volatile State state_;

    volatile float headingBuffer[1024];
    volatile int   leftPositionBuffer[1024];
    volatile int   rightPositionBuffer[1024];
    volatile float leftVelocityBuffer[1024];
    volatile float rightVelocityBuffer[1024];
    volatile int   logIndex;
    
    volatile float startHeading_;
    volatile float endHeading_;
    
};

#endif /* ROVER_H */
