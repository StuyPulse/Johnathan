/************************ PROJECT JON *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;


import com.stuypulse.robot.util.OV2311Camera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public interface Cameras {

    public static final CameraConfig DEFAULT_CAMERA = new CameraConfig("samera1",
            new Pose3d(-Units.inchesToMeters(12),
                       -Units.inchesToMeters(0),
                       +Units.inchesToMeters(5),
                       new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-30), Units.degreesToRadians(180))));

    public static final CameraConfig[] ROBOT_CAMERAS = new CameraConfig[]{DEFAULT_CAMERA};

    public static class CameraConfig {
        public final String NAME;
        public final Pose3d POSITION;

        public CameraConfig(String name, Pose3d position) {
            NAME = name;
            POSITION = position;
        }

        public OV2311Camera getCamera() {
            return new OV2311Camera(NAME, POSITION);
        }
    }
}
