/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.pathplanner.lib.util.PIDConstants;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Pose3d;
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
        double WIDTH = Units.inchesToMeters(26);
        double LENGTH = Units.inchesToMeters(26);

		SmartNumber MODULE_VELOCITY_DEADBAND = new SmartNumber("Swerve/Module velocity deadband (m per s)", 0.02);
		SmartNumber MAX_MODULE_SPEED = new SmartNumber("Swerve/Maximum module speed (m per s)", 5.06);
		SmartNumber MAX_MODULE_TURN = new SmartNumber("Swerve/Maximum module turn (rad per s)", 6.28);

        public interface Turn {
            SmartNumber kP = new SmartNumber("Swerve/Turn/kP", 6.0);
            SmartNumber kI = new SmartNumber("Swerve/Turn/kI", 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Turn/kD", 0.15);

            SmartNumber kV = new SmartNumber("Swerve/Turn/kV", 0.25);
            SmartNumber kA = new SmartNumber("Swerve/Turn/kA", 0.007);
        }

        public interface Drive {
            SmartNumber kP = new SmartNumber("Swerve/Drive/kP", 0.018327);
            SmartNumber kI = new SmartNumber("Swerve/Drive/kI", 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Drive/kD", 0.0);

            SmartNumber kS = new SmartNumber("Swerve/Drive/kS", 0.098993);
            SmartNumber kV = new SmartNumber("Swerve/Drive/kV", 2.4495);
            SmartNumber kA = new SmartNumber("Swerve/Drive/kA", 0.089872);
        }

        public interface Motion {
            
            PIDConstants XY = new PIDConstants(0.7, 0, 0.02);
            PIDConstants THETA = new PIDConstants(10, 0, 0.1);
        }
        
 		public interface FrontRight {
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(65.566406);
                // .plus(Rotation2d.fromDegrees(0));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * -0.5);
        }

        public interface FrontLeft {
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(47.197266);
                // .plus(Rotation2d.fromDegrees(270));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(208.212891);
                // .plus(Rotation2d.fromDegrees(180));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(154.511719);
                // .plus(Rotation2d.fromDegrees(90));
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

            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.1);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);

            SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Drive/Max Speed", Swerve.MAX_MODULE_SPEED.get());
            SmartNumber MAX_TELEOP_ACCEL = new SmartNumber("Driver Settings/Drive/Max Accleration", 15);
        }

        public interface Turn {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Turn/Deadband", 0.05);

            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.1);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);

            SmartNumber MAX_TELEOP_TURNING = new SmartNumber("Driver Settings/Turn/Max Turning", 6.0);

            public interface GyroFeedback {
                SmartBoolean GYRO_FEEDBACK_ENABLED = new SmartBoolean("Driver Settings/Gyro Feedback/Enabled", true);

                SmartNumber P = new SmartNumber("Driver Settings/Gyro Feedback/kP", 0.5);
                SmartNumber I = new SmartNumber("Driver Settings/Gyro Feedback/kI", 0.0);
                SmartNumber D = new SmartNumber("Driver Settings/Gyro Feedback/kD", 0.1);
            }

            public interface NoteAlignment {
                SmartNumber ANGLE_DEADBAND = new SmartNumber("Driver Settings/Note Alignment/Angle Deadband", 1);

                SmartNumber P = new SmartNumber("Driver Settings/Note Alignment/kP", 3.0);
                SmartNumber I = new SmartNumber("Driver Settings/Note Alignment/kI", 0.0);
                SmartNumber D = new SmartNumber("Driver Settings/Note Alignment/kD", 0.0);
            }
        }

    }

    public interface Intake {
        SmartNumber ACQUIRE_SPEED = new SmartNumber("Intake/Acquire Speed Bottom", 1);
        SmartNumber DEACQUIRE_SPEED = new SmartNumber("Intake/Deacquire Speed Bottom", -1);
    }


    public interface Limelight {
        String [] LIMELIGHTS = {
            "limelight"
        };

        int[] PORTS = {5800, 5801, 5802, 5803, 5804, 5805};
        Pose3d [] POSITIONS = new Pose3d[] {
            new Pose3d() // TODO: determine these values when Limelight is put on
        };
    }

    public static Vector2D vpow(Vector2D vec, double power) {
        return vec.mul(Math.pow(vec.magnitude(), power - 1));
    }

    public interface NoteDetection {
        SmartNumber DEBOUNCE_TIME = new SmartNumber("Note Detection/Debounce Time", 0.15);
        SmartNumber X_ANGLE_RC = new SmartNumber("Note Detection/X Angle RC", 0.05);
        SmartNumber TARGET_NOTE_DISTANCE = new SmartNumber("Note Detection/Target Note Distance", 0.5);

        SmartNumber THRESHOLD_X = new SmartNumber("Note Detection/X Threshold", 0.08);
        SmartNumber THRESHOLD_Y = new SmartNumber("Note Detection/Y Threshold", 0.1);
        SmartNumber THRESHOLD_ANGLE = new SmartNumber("Note Detection/Angle Threshold", 0.1);

        public interface Translation {
            SmartNumber P = new SmartNumber("Note Detection/Translation/kP", 1);
            SmartNumber I = new SmartNumber("Note Detection/Translation/kI", 0);
            SmartNumber D = new SmartNumber("Note Detection/Translation/kD", 0);
        }
        public interface Rotation {
            SmartNumber P = new SmartNumber("Note Detection/Rotation/kP", 1);
            SmartNumber I = new SmartNumber("Note Detection/Rotation/kI", 0);
            SmartNumber D = new SmartNumber("Note Detection/Rotation/kD", 0);
        }
    }
}