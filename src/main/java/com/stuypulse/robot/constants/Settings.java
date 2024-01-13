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

        SmartNumber PODIUM_RPM = new SmartNumber("FlywheelShooter/Podium RPM", 0);

        public interface PID {
            SmartNumber kP = new SmartNumber("FlywheelShooter kP", 0);
            SmartNumber kI = new SmartNumber("FlywheelShooter kI", 0);
            SmartNumber kD = new SmartNumber("FlywheelShooter kD", 0);
        }

        public interface FeedForward {
            SmartNumber kS = new SmartNumber("FlywheelShooter kS", 0);
            SmartNumber kV = new SmartNumber("FlywheelShooter kV", 0);
            SmartNumber kA = new SmartNumber("FlywheelShooter kA", 0);
        }
    }

    public interface TwoWheelShooter {

        SmartNumber PODIUM_RPM = new SmartNumber("TwoWheelShooter/Podium RPM", 0);

        public interface PID {
            SmartNumber kP = new SmartNumber("TwoWheelShooter kP", 0);
            SmartNumber kI = new SmartNumber("TwoWheelShooter kI", 0);
            SmartNumber kD = new SmartNumber("TwoWheelShooter kD", 0);
        }

        public interface FeedForward {
            SmartNumber kS = new SmartNumber("TwoWheelShooter kS", 0);
            SmartNumber kV = new SmartNumber("TwoWheelShooter kV", 0);
            SmartNumber kA = new SmartNumber("TwoWheelShooter kA", 0);
        }
    }
    public interface HorizontalShooter{

        SmartNumber PODIUM_RPM = new SmartNumber("HorizontalShooter/Podium RPM", 0);

        public interface PID {
            SmartNumber kP = new SmartNumber("HorizontalShooter kP", 0);
            SmartNumber kI = new SmartNumber("HorizontalShooter kI", 0);
            SmartNumber kD = new SmartNumber("HorizontalShooter kD", 0);
        }

        public interface FeedForward {
            SmartNumber kS = new SmartNumber("HorizontalShooter kS", 0);
            SmartNumber kV = new SmartNumber("HorizontalShooter kV", 0);
            SmartNumber kA = new SmartNumber("HorizontalShooter kA", 0);
    }
    }
}
