/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }

    public interface Shooter { //TO DO get ports
        int FLYWHEEL_MOTOR = 0; 
        int TWO_WHEEL_LEFT_MOTOR = 1;
        int TWO_WHEEL_RIGHT_MOTOR = 2;
        int HORIZONTAL_TOP_SHOOTER = 3;
        int HORIZONTAL_BOTTOM_SHOOTER = 4;
    }
}
