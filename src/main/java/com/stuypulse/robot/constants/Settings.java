/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.pathplanner.lib.util.PIDConstants;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    
    public interface Swerve {
        double WIDTH = 0.0;
        double LENGTH = 0.0;

        SmartNumber MAX_MODULE_SPEED = new SmartNumber("Max Module Speed (m/s)", 5);

        public interface Motion {
            PIDConstants XY = new PIDConstants(0.7, 0, 0.02);
            PIDConstants THETA = new PIDConstants(10, 0, 0.1);
        }
    }

    public interface Shooter {
        SmartNumber FLYWHEEL_PODIUM_RPM = new SmartNumber("FlywheelShooter/Podium RPM", 0);

        SmartNumber TWO_WHEEL_PODIUM_LEFT_RPM = new SmartNumber("TwoWheelShooter/Left Podium RPM", 0);
        SmartNumber TWO_WHEEL_PODIUM_RIGHT_RPM = new SmartNumber("TwoWheelShooter/Right Podium RPM", 0);

        SmartNumber HORIZONTAL_PODIUM_TOP_RPM = new SmartNumber("HorizontalShooter/Top Podium RPM", 0);
        SmartNumber HORIZONTAL_PODIUM_BOTTOM_RPM = new SmartNumber("HorizontalShooter/Bottom Podium RPM", 0);
        
        public interface FlyWheelShooter {
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
}
