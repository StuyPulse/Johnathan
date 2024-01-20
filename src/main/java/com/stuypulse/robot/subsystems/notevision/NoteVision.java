/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.notevision;

import static com.stuypulse.robot.constants.Settings.Limelight.*;

import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.Limelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NoteVision extends AbstractNoteVision {

    // store limelight network tables
    private final Limelight[] limelights;

    // store fieldobject2d's to display limelight data
    private final FieldObject2d[] limelightPoses;

    protected NoteVision() {
        // reference to all limelights on robot
        String[] hostNames = LIMELIGHTS;

        // setup limelight objects and field objects for april tag data
        limelights = new Limelight[hostNames.length];
        limelightPoses = new FieldObject2d[limelights.length];

        // setup all objects
        Field2d field = Odometry.getInstance().getField();
        for(int i = 0; i < hostNames.length; i++){
            limelights[i] = new Limelight(hostNames[i], POSITIONS[i]);
            limelightPoses[i] = field.getObject(hostNames[i] + " pose");

            for (int port : PORTS) {
                PortForwarder.add(port + i * 10, hostNames[i] + ".local", port);
            }
        }
    }

    @Override
    public boolean hasNoteData() {
        for (Limelight limelight : limelights) {
            if (limelight.hasNoteData())
                return true;
        }

        return false;
    }

    @Override
    public double getDistanceToNote() {
        double distance = 0;

        for (Limelight limelight : limelights) {
            if (limelight.hasNoteData()) {
                distance +=  limelight.getDistanceToNote();
            }
        }

        return distance / limelights.length;
    }

    @Override
    public Rotation2d getRotationToNote() {
        double degrees = 0;

        for (Limelight limelight : limelights) {
            degrees += limelight.getXAngle();
        }

        return Rotation2d.fromDegrees(degrees / limelights.length);
    }

    @Override
    public void periodic() {
        for (int i = 0; i < limelights.length; ++i) {
            limelights[i].updateData();
        }

        if (hasNoteData()) {
            SmartDashboard.putNumber("Note Detection/X Angle", getRotationToNote().getDegrees());
            SmartDashboard.putNumber("Note Detection/Y Angle", limelights[0].getYAngle());
            SmartDashboard.putNumber("Note Detection/Note Distance", getDistanceToNote());
        }
    }
}