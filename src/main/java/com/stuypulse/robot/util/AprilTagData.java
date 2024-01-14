/************************ PROJECT JON *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AprilTagData {
    public final Pose2d pose;
    public final double latency;
    public final int id;
    public final Pose3d cameraPose;

    private static double getDegreesBetween(Rotation2d a, Rotation2d b) {
        double c = a.getCos() * b.getCos() + a.getSin() * b.getSin();
        double d = (1 - c) * 180;

        return d;
    }

    public double getDegreesToTag() {
        Rotation2d tag = Field.getFiducialFromID(id).getPose().getRotation().toRotation2d();

        return getDegreesBetween(tag.plus(Rotation2d.fromDegrees(180)), pose.getRotation().minus(cameraPose.getRotation().toRotation2d()));
    }

    public double getDistanceToTag() {
        Translation2d robot = pose.getTranslation();
        Translation2d tag = Field.getFiducialFromID(id).getPose().getTranslation().toTranslation2d();

        return robot.getDistance(tag);
    }

    /**
     * Record april tag data, usually from a vision system
     *
     * @param robotPose robot pose information, in meters with blue/red corner as origin
     * @param latency total latency, in seconds
     * @param id tag id
     * @param cameraPose associated camera's robot relative pose
     */
    public AprilTagData(Pose2d robotPose, double latency, int id, Pose3d cameraPose) {
        this.pose = robotPose;
        this.latency = latency;
        this.id = id;
        this.cameraPose = cameraPose;
    }
}
