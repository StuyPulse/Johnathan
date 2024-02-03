/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.notevision;

import static com.stuypulse.robot.constants.Settings.Limelight.*;

import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NoteVision extends AbstractNoteVision {

    // store limelight network tables
    private final Limelight[] limelights;

    // store fieldobject2d's to display limelight data
    private final FieldObject2d[] limelightPoses;

    private Translation2d notePose;

    private FieldObject2d note2d;

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

        note2d = AbstractOdometry.getInstance().getField().getObject("Note");

        notePose = new Translation2d();
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
    public Translation2d getEstimatedNoteTranslation() {
        return notePose;
    }

    public void updateNotePose() {
        Translation2d sum = new Translation2d();

        for (Limelight limelight : limelights) {
            AbstractOdometry odometry = AbstractOdometry.getInstance();

            Translation2d limelightToNote = new Translation2d(limelight.getDistanceToNote(), Rotation2d.fromDegrees(limelight.getXAngle()));

            Translation2d robotToNote = limelightToNote
                .minus(limelight.robotRelativePose.getTranslation().toTranslation2d())
                .rotateBy(limelight.robotRelativePose.getRotation().toRotation2d());

            Translation2d fieldToNote = robotToNote.rotateBy(odometry.getRotation()).plus(odometry.getTranslation());

            sum = sum.plus(fieldToNote);
        }

        notePose = sum.div(limelights.length);
    }

    @Override
    public void periodic() {
        for (int i = 0; i < limelights.length; ++i) {
            limelights[i].updateData();
        }

        note2d.setPose(new Pose2d(notePose, new Rotation2d()));

        if (hasNoteData()) {
            updateNotePose();

            SmartDashboard.putNumber("Note Detection/X Angle", limelights[0].getXAngle());
            SmartDashboard.putNumber("Note Detection/Y Angle", limelights[0].getYAngle());
            SmartDashboard.putNumber("Note Detection/Distance", limelights[0].getDistanceToNote());
            SmartDashboard.putNumber("Note Detection/Estimated X", notePose.getX());
            SmartDashboard.putNumber("Note Detection/Estimated Y", notePose.getY());
        }
    }
}