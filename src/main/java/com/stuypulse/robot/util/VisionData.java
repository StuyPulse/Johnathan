/************************ PROJECT JON *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose3d;

public class VisionData {

    public final long[] tids;
    public final Pose3d cameraLocation;
    public Pose3d robotPose;
    public final double timestamp;

    public double calculateDistanceToTag(Fiducial fiducial) {
        return robotPose.getTranslation().getDistance(fiducial.getPose().getTranslation());
    }

    private int getPrimaryTID() {
        if (tids.length == 0) return -1;
        return (int) tids[0];
    }

    public Fiducial getPrimaryFiducial() {
        return Field.getFiducial(getPrimaryTID());
    }

    public VisionData(long[] tids, Pose3d cameraLocation, Pose3d robotPose, double timestamp) {
        this.tids = tids;
        this.cameraLocation = cameraLocation;
        this.robotPose = robotPose;
        this.timestamp = timestamp;
    }
}
