/************************ PROJECT JON *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

import java.util.ArrayList;

import com.stuypulse.robot.util.Fiducial;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public interface Field {

    double WIDTH = 16.54;
    double HEIGHT = 8.02;

    public static final double FIDUCIAL_SIZE = 0.15716;

    Fiducial FIDUCIALS[] = {
        new Fiducial(1,new Pose3d(new Translation3d(WIDTH / 2, HEIGHT / 2, Units.inchesToMeters(30)), new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(0),Units.degreesToRadians(0)))),
        new Fiducial(3,new Pose3d(new Translation3d(WIDTH / 2, HEIGHT / 2 - Units.inchesToMeters(44.25), Units.inchesToMeters(30)), new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(0),Units.degreesToRadians(0)))),
    };

    public static Fiducial[] getFiducialLayout(int[] tids) {
        ArrayList<Fiducial> fiducials = new ArrayList<Fiducial>();
        for (int tid : tids) for (Fiducial fiducial : FIDUCIALS) if (fiducial.getID() == tid) fiducials.add(fiducial);
        Fiducial[] fiducials_array = new Fiducial[fiducials.size()];
        return fiducials.toArray(fiducials_array);
    }

    public static boolean isValidTag(int id) {
        for (Fiducial fiducial : FIDUCIALS)
            if (fiducial.getID() == id) return true;
        return false;
    }

    public static Fiducial getTag(int id) {
        for (Fiducial fiducial : FIDUCIALS)
            if (fiducial.getID() == id) return fiducial;
        return null;
    }

    public static double[] getTagLayout(Fiducial[] fiducials) {
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

    public static double[] getTagPoses(Fiducial[] fiducials) {
        double[] layout = new double[fiducials.length * 3];

        for (int i = 0; i < fiducials.length; i++) {
            Fiducial tag = fiducials[i];
            layout[i * 3 + 0] = tag.getPose().getTranslation().getX();
            layout[i * 3 + 1] = tag.getPose().getTranslation().getY();
            layout[i * 3 + 2] = Units.radiansToDegrees(tag.getPose().getRotation().getZ());
        }

        return layout;
    }

    public static Fiducial getFiducialFromID(int tid) {
        for (Fiducial fiducial : FIDUCIALS) if (fiducial.getID() == tid) return fiducial;
        return null;
    }
}