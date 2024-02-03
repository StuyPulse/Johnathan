/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.pathplanner.lib.util.PIDConstants;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public enum RobotType {
        JIM("03262B9F"),           // DeviceCode=0x7AAE@EDITOR=vi@PWD=/@TERM=dumb@DeviceDesc=roboRIO 2.0@SHLVL=3@TargetClass=cRIO@serialnum=03262B9F@PATH=/usr/local/bin:/usr/bin:/bin:/usr/local/frc/bin:/usr/local/natinst/bin@FPGADeviceCode=0x7AAF@_=/usr/local/frc/JRE/bin/java
        OFFSEASON_BOT("0305a69d"), // DeviceCode=0x76F2@PWD=/usr/local/natinst/labview@DeviceDesc=roboRIO@SHLVL=2@TargetClass=cRIO@serialnum=0305a69d@FPGADeviceCode=0x77A9@_=/sbin/start-stop-daemon@
        SIM("");

        public final String serialNum;

        RobotType(String serialNum) {
            this.serialNum = serialNum;
        }

        public static RobotType fromString(String serialNum) {
            for (RobotType robot : RobotType.values()) {
                if (robot.serialNum.equals(serialNum)) {
                    return robot;
                }
            }

            return RobotType.SIM;
        }
    }

    double DT = 0.02;

	public interface Swerve {
        double WIDTH = Units.inchesToMeters(21);
        double LENGTH = Units.inchesToMeters(21);

		SmartNumber MODULE_VELOCITY_DEADBAND = new SmartNumber("Swerve/Module velocity deadband (m per s)", 0.02);
		double MAX_MODULE_SPEED = 5.88;

        public interface Turn {
            double kP = 6.0;
            double kI = 0.0;
            double kD = 0.15;

            double kS = 0.44076;
            double kV = 0.0056191;
            double kA = 0.00042985;
        }

        public interface Drive {
            double kP = 0.002794;
            double kI = 0.0;
            double kD = 0.0;

            double kS = 0.17313;
            double kV = 1.7573 + 0.1;
            double kA = 0.19554 + 0.1;
        }

        public interface Motion {
            PIDConstants XY = new PIDConstants(1, 0, 0.02);
            PIDConstants THETA = new PIDConstants(10, 0, 0.1);
        }
        
 		public interface FrontRight {
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(-153.632812 + 180);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * -0.5);
        }

        public interface FrontLeft {
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(147.919922 + 180);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(73.125 + 180);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(-2.02184 + 180);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * -0.5);
        }

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(4);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
                double GEAR_RATIO = 1.0 / 6.12;

                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
        }
	}
    
    public interface Driver {
        public interface Drive {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Drive/Deadband", 0.03);

            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.01);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);

            SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Drive/Max Speed", Swerve.MAX_MODULE_SPEED);
            SmartNumber MAX_TELEOP_ACCEL = new SmartNumber("Driver Settings/Drive/Max Accleration", 15);
        }

        public interface Turn {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Turn/Deadband", 0.05);

            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.1);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);

            SmartNumber MAX_TELEOP_TURNING = new SmartNumber("Driver Settings/Turn/Max Turning", 6.28);

            public interface GyroFeedback {
                SmartBoolean GYRO_FEEDBACK_ENABLED = new SmartBoolean("Driver Settings/Gyro Feedback/Enabled", true);

                SmartNumber P = new SmartNumber("Driver Settings/Gyro Feedback/kP", 0.5);
                SmartNumber I = new SmartNumber("Driver Settings/Gyro Feedback/kI", 0.0);
                SmartNumber D = new SmartNumber("Driver Settings/Gyro Feedback/kD", 0.1);
            }
        }

    }

    public static Vector2D vpow(Vector2D vec, double power) {
        return vec.mul(Math.pow(vec.magnitude(), power - 1));
    }

    public interface Intake {
        SmartNumber ACQUIRE_SPEED_TOP = new SmartNumber("Intake/Acquire Speed Top", 1);
        SmartNumber ACQUIRE_SPEED_BOTTOM = new SmartNumber("Intake/Acquire Speed Bottom", 1);

        SmartNumber DEACQUIRE_SPEED_TOP = new SmartNumber("Intake/Deacquire Speed Top", -1);
        SmartNumber DEACQUIRE_SPEED_BOTTOM = new SmartNumber("Intake/Deacquire Speed Bottom", -1);
    }

    public interface Alignment {

        SmartNumber DEBOUNCE_TIME = new SmartNumber("Alignment/Debounce Time", 0.15);
        SmartNumber X_TOLERANCE = new SmartNumber("Alignment/X Tolerance", 0.1);
        SmartNumber Y_TOLERANCE = new SmartNumber("Alignment/Y Tolerance", 0.1);
        SmartNumber ANGLE_TOLERANCE = new SmartNumber("Alignment/Angle Tolerance", 5);

        SmartNumber TARGET_DISTANCE_IN = new SmartNumber("Alignment/Target Distance (in)", 110);
        SmartNumber TAKEOVER_DISTANCE_IN = new SmartNumber("Alignment/Takeover Distance (in)", 50);

        public interface Translation {
            SmartNumber P = new SmartNumber("Alignment/Translation/kP", 2.5);
            SmartNumber I = new SmartNumber("Alignment/Translation/kI", 0);
            SmartNumber D = new SmartNumber("Alignment/Translation/kD", 0.0);
        }

        public interface Rotation {
            SmartNumber P = new SmartNumber("Alignment/Rotation/kP", 1);
            SmartNumber I = new SmartNumber("Alignment/Rotation/kI", 0);
            SmartNumber D = new SmartNumber("Alignment/Rotation/kD", 0);
        }

        public interface Gyro {
            SmartNumber P = new SmartNumber("Alignment/Gyro/kP", 12);
            SmartNumber I = new SmartNumber("Alignment/Gyro/kI", 0);
            SmartNumber D = new SmartNumber("Alignment/Gyro/kD", 0.1);
        }
    }

    public interface LED {
        int LED_LENGTH = 55;
        SmartNumber MANUAL_UPDATE_TIME = new SmartNumber("LED/LED Manual Update Time", .75);
        SmartNumber BLINK_TIME = new SmartNumber("LED/LED Blink Time", .5);

        SmartNumber TRANSLATION_SPREAD = new SmartNumber("LED/LED Translation Spread (m)", 1);
        SmartNumber ROTATION_SPREAD = new SmartNumber("LED/LED Rotation Spread (deg)", 35);
    }
}
