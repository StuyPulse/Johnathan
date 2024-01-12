/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public interface FlyWheelShooter {
        public interface PID {
            SmartNumber kP = new SmartNumber("Shooter kP", 0);
            SmartNumber kI = new SmartNumber("Shooter kI", 0);
            SmartNumber kD = new SmartNumber("Shooter kD", 0);
        }

        public interface FeedForward {
            SmartNumber kS = new SmartNumber("Shooter kS", 0);
            SmartNumber kV = new SmartNumber("Shooter kV", 0);
            SmartNumber kA = new SmartNumber("Shooter kA", 0);
        }
    }

    public interface TwoWheelShooter {
        public interface PID {
            SmartNumber kP = new SmartNumber("Shooter kP", 0);
            SmartNumber kI = new SmartNumber("Shooter kI", 0);
            SmartNumber kD = new SmartNumber("Shooter kD", 0);
        }

        public interface FeedForward {
            SmartNumber kS = new SmartNumber("Shooter kS", 0);
            SmartNumber kV = new SmartNumber("Shooter kV", 0);
            SmartNumber kA = new SmartNumber("Shooter kA", 0);
        }
    }
}
