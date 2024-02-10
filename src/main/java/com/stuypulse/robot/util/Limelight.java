/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import static com.stuypulse.robot.constants.Settings.Limelight.LIMELIGHTS;
import static com.stuypulse.robot.constants.Settings.Limelight.POSITIONS;

import com.stuypulse.robot.constants.Settings.NoteDetection;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight {

    private final String tableName;
    
    private final DoubleEntry txEntry;
    private final DoubleEntry tyEntry;
    private final IntegerEntry tvEntry;

    private int limelightId;

    private double txData;
    private double tyData;

    private IStream xAngle;
    private BStream noteData;

    public final Pose3d robotRelativePose;

    public Limelight(String tableName, Pose3d robotRelativePose) {
        this.tableName = tableName;
        this.robotRelativePose = robotRelativePose;

        for(int i = 0; i < LIMELIGHTS.length; i++) {
            if(LIMELIGHTS[i].equals(tableName)) {
                limelightId = i;
            }
        }

        NetworkTable limelight = NetworkTableInstance.getDefault().getTable(tableName);


        txEntry = limelight.getDoubleTopic("tx").getEntry(0.0);
        tyEntry = limelight.getDoubleTopic("ty").getEntry(0.0);
        tvEntry = limelight.getIntegerTopic("tv").getEntry(0);

        xAngle = IStream.create(() -> txData)
            .filtered(new LowPassFilter(NoteDetection.X_ANGLE_RC));
        noteData = BStream.create(() -> tvEntry.get() == 1)
            .filtered(new BDebounceRC.Rising(NoteDetection.DEBOUNCE_TIME));
    }

    public String getTableName() {
        return tableName;
    }

    public boolean hasNoteData() {
        return noteData.get();
    }

    public void updateData() {
        if (tvEntry.get() == 1) {
            txData = txEntry.get();
            tyData = tyEntry.get();
        }
    }

    public double getXAngle() {
        return -xAngle.get() + Units.radiansToDegrees(POSITIONS[limelightId].getRotation().getZ());
    }

    public double getYAngle() {
        return tyData - Units.radiansToDegrees(POSITIONS[limelightId].getRotation().getY());
    }

    public double getDistanceToNote() {
        Rotation2d yRotation =  Rotation2d.fromDegrees(getYAngle());
        return POSITIONS[limelightId].getZ() / -yRotation.getTan() + POSITIONS[limelightId].getX() - Units.inchesToMeters(14.0 / 2.0);
    }
}