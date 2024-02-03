/************************ PROJECT JON *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

import java.util.ArrayList;

import com.stuypulse.robot.util.Fiducial;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public interface Field {

    double WIDTH = 16.54;
    double HEIGHT = 8.02;

    double NOTE_LENGTH = Units.inchesToMeters(14.0);
    
    public static final double FIDUCIAL_SIZE = 0.15716;

    Fiducial FIDUCIALS[] = {
        // 2024 Field Fiducial Layout
        new Fiducial(1,  new Pose3d(new Translation3d(Units.inchesToMeters(593.68), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(120)))),
        new Fiducial(2,  new Pose3d(new Translation3d(Units.inchesToMeters(637.21), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(120)))),
        new Fiducial(3,  new Pose3d(new Translation3d(Units.inchesToMeters(652.73), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new Fiducial(4,  new Pose3d(new Translation3d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new Fiducial(5,  new Pose3d(new Translation3d(Units.inchesToMeters(578.77), Units.inchesToMeters(323.0), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
        new Fiducial(6,  new Pose3d(new Translation3d(Units.inchesToMeters(72.5), Units.inchesToMeters(323.0), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
        new Fiducial(7,  new Pose3d(new Translation3d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new Fiducial(8,  new Pose3d(new Translation3d(Units.inchesToMeters(-1.5), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new Fiducial(9,  new Pose3d(new Translation3d(Units.inchesToMeters(14.02), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(60)))),
        new Fiducial(10, new Pose3d(new Translation3d(Units.inchesToMeters(57.54), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(60)))),
        new Fiducial(11, new Pose3d(new Translation3d(Units.inchesToMeters(468.69), Units.inchesToMeters(146.19), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(300)))),
        new Fiducial(12, new Pose3d(new Translation3d(Units.inchesToMeters(468.69), Units.inchesToMeters(177.10), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(60)))),
        new Fiducial(13, new Pose3d(new Translation3d(Units.inchesToMeters(441.74), Units.inchesToMeters(161.62), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new Fiducial(14, new Pose3d(new Translation3d(Units.inchesToMeters(209.48), Units.inchesToMeters(161.62), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new Fiducial(15, new Pose3d(new Translation3d(Units.inchesToMeters(182.73), Units.inchesToMeters(177.10), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(120)))),
        new Fiducial(16, new Pose3d(new Translation3d(Units.inchesToMeters(182.73), Units.inchesToMeters(146.19), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(240)))),
    };

    public static Fiducial[] getFiducialLayout(int[] tids) {
        ArrayList<Fiducial> fiducials = new ArrayList<Fiducial>();
        for (int tid : tids) for (Fiducial fiducial : FIDUCIALS) if (fiducial.getID() == tid) fiducials.add(fiducial);
        Fiducial[] fiducials_array = new Fiducial[fiducials.size()];
        return fiducials.toArray(fiducials_array);
    }

    public static double[] getFiducialLayoutAsDoubleArray(Fiducial[] fiducials) {
        double[] layout = new double[fiducials.length * 7];

        for (int i = 0; i < fiducials.length; i++) {
            Fiducial fiducial = fiducials[i];
            layout[i * 7 + 0] = fiducial.getID();
            layout[i * 7 + 1] = fiducial.getPose().getTranslation().getX();
            layout[i * 7 + 2] = fiducial.getPose().getTranslation().getY();
            layout[i * 7 + 3] = fiducial.getPose().getTranslation().getZ();
            layout[i * 7 + 4] = fiducial.getPose().getRotation().getX();
            layout[i * 7 + 5] = fiducial.getPose().getRotation().getY();
            layout[i * 7 + 6] = fiducial.getPose().getRotation().getZ();
        }

        return layout;
    }

    public static Fiducial getFiducial(int id) {
        for (Fiducial fiducial : FIDUCIALS)
            if (fiducial.getID() == id) return fiducial;
        return null;
    }

    public static double[] getFiducialPosesAsDoubleArray(Fiducial[] fiducials) {
        double[] layout = new double[fiducials.length * 3];

        for (int i = 0; i < fiducials.length; i++) {
            Fiducial fiducial = fiducials[i];
            layout[i * 3 + 0] = fiducial.getPose().getTranslation().getX();
            layout[i * 3 + 1] = fiducial.getPose().getTranslation().getY();
            layout[i * 3 + 2] = Units.radiansToDegrees(fiducial.getPose().getRotation().getZ());
        }

        return layout;
    }

    public static Fiducial getFiducialFromID(int tid) {
        for (Fiducial fiducial : FIDUCIALS) if (fiducial.getID() == tid) return fiducial;
        return null;
    }

    public static boolean isValidFiducialID(int id) {
        for (Fiducial fiducial : FIDUCIALS)
            if (fiducial.getID() == id) return true;
        return false;
    }

    Pose2d SPEAKER_POSES[] = {
        getFiducial(7).getPose().toPose2d(), // BLUE
        getFiducial(4).getPose().toPose2d(), // RED
    };

    public static Pose2d getSpeakerPose() {
        boolean isBlue = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
        return SPEAKER_POSES[isBlue ? 0 : 1];
    }
}