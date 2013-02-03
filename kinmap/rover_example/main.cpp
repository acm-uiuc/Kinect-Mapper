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
 * Rover example.
 * 
 * Reads and executes a set of commands from a simple command script.
 */

/**
 * Includes
 */
#include "Rover.h"

//Debugging.
Serial pc(USBTX, USBRX);
DigitalOut led1(LED1);

//Globals.
LocalFileSystem local("local");
SDFileSystem sd(p11, p12, p13, p14, "sd");

int main() {

    //left motor pwm, left motor brake, left motor direction,
    //right motor pwm, right motor brake, right motor direction,
    //left qei A, left qei B, left qei index, left pulses per rev,
    //right qei A, right qei B, right qei index, right pulses per rev.
    Rover myRover(p23, p19, p28, p26, p22, p24,
                  p29, p30, NC, 624,
                  p18, p21, NC, 624);

    //-----------------------
    // Simple command parser
    //-----------------------
    char  cmd0[16]; //{"move", "turn"}
    char  cmd1[16]; //{{"forward", "backward"}, {"left", "right"}}
    float cmd2 = 0; //{distance in METRES, angle in DEGREES}

    pc.printf("Starting...\n");

    //Wait a bit before we start moving.
    wait(3);

    //Open the command script.
    FILE *fp = fopen("/local/commands.txt", "r");

    //Check if we were successful in opening the command script.
    if (fp == NULL) {
        pc.printf("Opening file failed...\n");
    } else {
        pc.printf("Opening file succeeded!\n");
    }

    //While there's another line to read from the command script.
    while (fscanf(fp, "%s%s%f", cmd0, cmd1, &cmd2) >= 0) {
    
        led1 = 1;
    
        pc.printf("Start of new command\n");

        wait(1);

        pc.printf("%s %s %f\n", cmd0, cmd1, cmd2);

        //move command.
        if (strcmp(cmd0, "move") == 0) {
            if (strcmp(cmd1, "forward") == 0) {
                myRover.move(cmd2);
                while (myRover.getState() != Rover::STATE_STATIONARY) {
                
                }
                pc.printf("Finished!\n");
            } else if (strcmp(cmd1, "backward") == 0) {
                myRover.move(-cmd2);
                while (myRover.getState() != Rover::STATE_STATIONARY) {
                }
            }
        }
        //turn command.
        else if (strcmp(cmd0, "turn") == 0) {
            if (strcmp(cmd1, "left") == 0) {
                myRover.turn(-cmd2);
                while (myRover.getState() != Rover::STATE_STATIONARY) {
                }
            } else if (strcmp(cmd1, "right") == 0) {
                myRover.turn(cmd2);
                while (myRover.getState() != Rover::STATE_STATIONARY) {
                }
            }
        }
        
        pc.printf("End of command\n");
        led1 = 0;

    }

    wait(1);

    myRover.stopLogging();
    fclose(fp);

}
