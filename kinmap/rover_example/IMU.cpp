/**
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
 * IMU consisting of ADXL345 accelerometer and ITG-3200 gyroscope using
 * orientation filter developed by Sebastian Madgwick.
 *
 * Find more details about his paper here:
 *
 * http://code.google.com/p/imumargalgorithm30042010sohm/
 */


/**
 * Includes
 */
#include "IMU.h"

IMU::IMU(float imuRate,
         double gyroscopeMeasurementError,
         float accelerometerRate,
         float gyroscopeRate) : accelerometer(p5, p6, p7, p8),
        gyroscope(p9, p10), imuFilter(imuRate, gyroscopeMeasurementError) {

    imuRate_ = imuRate;
    accelerometerRate_ = accelerometerRate;
    gyroscopeRate_ = gyroscopeRate;

    //Initialize sampling variables.
    a_xAccumulator = 0;
    a_yAccumulator = 0;
    a_zAccumulator = 0;
    w_xAccumulator = 0;
    w_yAccumulator = 0;
    w_zAccumulator = 0;

    accelerometerSamples = 0;
    gyroscopeSamples = 0;

    //Initialize and calibrate sensors.
    initializeAccelerometer();
    calibrateAccelerometer();

    initializeGyroscope();
    calibrateGyroscope();

    //To reduce the number of interrupts we'll remove the separate tickers for
    //the accelerometer, gyro and filter update and combine them all into one.

    //accelerometerTicker.attach(this, &IMU::sampleAccelerometer, accelerometerRate_);
    //gyroscopeTicker.attach(this, &IMU::sampleGyroscope, gyroscopeRate_);
    sampleTicker.attach(this, &IMU::sample, accelerometerRate_);
    //filterTicker.attach(this, &IMU::filter, imuRate_);

}

double IMU::getRoll(void) {

    return toDegrees(imuFilter.getRoll());

}

double IMU::getPitch(void) {

    return toDegrees(imuFilter.getPitch());

}

double IMU::getYaw(void) {

    return toDegrees(imuFilter.getYaw());

}

void IMU::initializeAccelerometer(void) {

    //Go into standby mode to configure the device.
    accelerometer.setPowerControl(0x00);
    //Full resolution, +/-16g, 4mg/LSB.
    accelerometer.setDataFormatControl(0x0B);
    //200Hz data rate.
    accelerometer.setDataRate(ADXL345_200HZ);
    //Measurement mode.
    accelerometer.setPowerControl(0x08);
    //See http://www.analog.com/static/imported-files/application_notes/AN-1077.pdf
    wait_ms(22);

}

void IMU::sampleAccelerometer(void) {

    //If we've taken a certain number of samples,
    //average them, remove the bias and convert the units.
    if (accelerometerSamples == SAMPLES) {

        a_x = ((a_xAccumulator / SAMPLES) - a_xBias) * ACCELEROMETER_GAIN;
        a_y = ((a_yAccumulator / SAMPLES) - a_yBias) * ACCELEROMETER_GAIN;
        a_z = ((a_zAccumulator / SAMPLES) - a_zBias) * ACCELEROMETER_GAIN;

        a_xAccumulator = 0;
        a_yAccumulator = 0;
        a_zAccumulator = 0;
        accelerometerSamples = 0;

    }
    //Otherwise, accumulate another reading. 
    else {

        accelerometer.getOutput(readings);

        a_xAccumulator += (int16_t) readings[0];
        a_yAccumulator += (int16_t) readings[1];
        a_zAccumulator += (int16_t) readings[2];

        accelerometerSamples++;

    }

}

void IMU::calibrateAccelerometer(void) {

    a_xAccumulator = 0;
    a_yAccumulator = 0;
    a_zAccumulator = 0;

    //Accumulate a certain number of samples.
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {

        accelerometer.getOutput(readings);

        a_xAccumulator += (int16_t) readings[0];
        a_yAccumulator += (int16_t) readings[1];
        a_zAccumulator += (int16_t) readings[2];

        wait(accelerometerRate_);

    }

    //Average the samples.
    a_xAccumulator /= CALIBRATION_SAMPLES;
    a_yAccumulator /= CALIBRATION_SAMPLES;
    a_zAccumulator /= CALIBRATION_SAMPLES;

    //These are our zero g offsets.
    //250 = 9.81m/s/s @ 4mg/LSB.
    a_xBias = a_xAccumulator;
    a_yBias = a_yAccumulator;
    a_zBias = (a_zAccumulator - 250);

    //Reset accumulators.
    a_xAccumulator = 0;
    a_yAccumulator = 0;
    a_zAccumulator = 0;

}

void IMU::initializeGyroscope(void) {

    //Low pass filter bandwidth of 42Hz.
    gyroscope.setLpBandwidth(LPFBW_42HZ);
    //Internal sample rate of 200Hz.
    gyroscope.setSampleRateDivider(4);

}

void IMU::calibrateGyroscope(void) {

    w_xAccumulator = 0;
    w_yAccumulator = 0;
    w_zAccumulator = 0;

    //Accumulate a certain number of samples.
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {

        w_xAccumulator += gyroscope.getGyroX();
        w_yAccumulator += gyroscope.getGyroY();
        w_zAccumulator += gyroscope.getGyroZ();
        wait(gyroscopeRate_);

    }

    //Average the samples.
    w_xAccumulator /= CALIBRATION_SAMPLES;
    w_yAccumulator /= CALIBRATION_SAMPLES;
    w_zAccumulator /= CALIBRATION_SAMPLES;

    //Set the null bias.
    w_xBias = w_xAccumulator;
    w_yBias = w_yAccumulator;
    w_zBias = w_zAccumulator;

    //Reset the accumulators.
    w_xAccumulator = 0;
    w_yAccumulator = 0;
    w_zAccumulator = 0;

}

void IMU::sampleGyroscope(void) {

    //If we've taken the required number of samples then,
    //average the samples, removed the null bias and convert the units
    //to rad/s.
    if (gyroscopeSamples == SAMPLES) {

        w_x = toRadians(((w_xAccumulator / SAMPLES) - w_xBias) * GYROSCOPE_GAIN);
        w_y = toRadians(((w_yAccumulator / SAMPLES) - w_yBias) * GYROSCOPE_GAIN);
        w_z = toRadians(((w_zAccumulator / SAMPLES) - w_zBias) * GYROSCOPE_GAIN);

        w_xAccumulator = 0;
        w_yAccumulator = 0;
        w_zAccumulator = 0;
        gyroscopeSamples = 0;

    }
    //Accumulate another sample. 
    else {

        w_xAccumulator += gyroscope.getGyroX();
        w_yAccumulator += gyroscope.getGyroY();
        w_zAccumulator += gyroscope.getGyroZ();

        gyroscopeSamples++;

    }

}

void IMU::sample(void) {

    //If we've taken enough samples then,
    //average the samples, remove the offsets and convert to appropriate units.
    //Feed this information into the filter to calculate the new Euler angles.
    if (accelerometerSamples == SAMPLES) {

        a_x = ((a_xAccumulator / SAMPLES) - a_xBias) * ACCELEROMETER_GAIN;
        a_y = ((a_yAccumulator / SAMPLES) - a_yBias) * ACCELEROMETER_GAIN;
        a_z = ((a_zAccumulator / SAMPLES) - a_zBias) * ACCELEROMETER_GAIN;

        a_xAccumulator = 0;
        a_yAccumulator = 0;
        a_zAccumulator = 0;

        accelerometerSamples = 0;

        w_x = toRadians(((w_xAccumulator / SAMPLES) - w_xBias) * GYROSCOPE_GAIN);
        w_y = toRadians(((w_yAccumulator / SAMPLES) - w_yBias) * GYROSCOPE_GAIN);
        w_z = toRadians(((w_zAccumulator / SAMPLES) - w_zBias) * GYROSCOPE_GAIN);

        w_xAccumulator = 0;
        w_yAccumulator = 0;
        w_zAccumulator = 0;
        gyroscopeSamples = 0;

        //Update the filter variables.
        imuFilter.updateFilter(w_y, w_x, w_z, a_y, a_x, a_z);
        //Calculate the new Euler angles.
        imuFilter.computeEuler();

    }
    //Accumulate another sample. 
    else {

        accelerometer.getOutput(readings);

        a_xAccumulator += (int16_t) readings[0];
        a_yAccumulator += (int16_t) readings[1];
        a_zAccumulator += (int16_t) readings[2];

        w_xAccumulator += gyroscope.getGyroX();
        w_yAccumulator += gyroscope.getGyroY();
        w_zAccumulator += gyroscope.getGyroZ();

        accelerometerSamples++;

    }

}

void IMU::filter(void) {

    //Update the filter variables.
    imuFilter.updateFilter(w_y, w_x, w_z, a_y, a_x, a_z);
    //Calculate the new Euler angles.
    imuFilter.computeEuler();

}

void IMU::reset(void) {

    //Disable interrupts.
    sampleTicker.detach();
    
    //Recalibrate sensors.
    calibrateAccelerometer();
    calibrateGyroscope();
    
    //Reset the IMU filter.
    imuFilter.reset();
    
    //Reset the working variables.
    a_xAccumulator = 0;
    a_yAccumulator = 0;
    a_zAccumulator = 0;
    w_xAccumulator = 0;
    w_yAccumulator = 0;
    w_zAccumulator = 0;
    accelerometerSamples = 0;
    gyroscopeSamples = 0;
    
    //Enable interrupts.
    sampleTicker.attach(this, &IMU::sample, accelerometerRate_);

}
