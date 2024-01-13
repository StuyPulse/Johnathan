/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public interface Intake {
        SmartNumber ACQUIRE_SPEED_TOP = new SmartNumber("Intake/Acquire Speed Top", 1);
        SmartNumber ACQUIRE_SPEED_BOTTOM = new SmartNumber("Intake/Acquire Speed Bottom", 1);

        SmartNumber DEACQUIRE_SPEED_TOP = new SmartNumber("Intake/Deacquire Speed Top", -1);
        SmartNumber DEACQUIRE_SPEED_BOTTOM = new SmartNumber("Intake/Deacquire Speed Bottom", -1);

    }
}
