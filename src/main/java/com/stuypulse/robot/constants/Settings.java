/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public enum RobotType {
        JIM("03262B9F"),           // DeviceCode=0x7AAE@EDITOR=vi@PWD=/@TERM=dumb@DeviceDesc=roboRIO 2.0@SHLVL=3@TargetClass=cRIO@serialnum=03262B9F@PATH=/usr/local/bin:/usr/bin:/bin:/usr/local/frc/bin:/usr/local/natinst/bin@FPGADeviceCode=0x7AAF@_=/usr/local/frc/JRE/bin/java
        OFFSEASON_BOT("0305a69d"); // DeviceCode=0x76F2@PWD=/usr/local/natinst/labview@DeviceDesc=roboRIO@SHLVL=2@TargetClass=cRIO@serialnum=0305a69d@FPGADeviceCode=0x77A9@_=/sbin/start-stop-daemon@

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

            return null;
        }
    }

    double DT = 0.02;
}
