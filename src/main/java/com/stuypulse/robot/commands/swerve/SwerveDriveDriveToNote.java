/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.swerve;

import static com.stuypulse.robot.constants.Settings.NoteDetection.*;

import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.notevision.AbstractNoteVision;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDriveToNote extends Command {

    private final SwerveDrive swerve;
    private final AbstractNoteVision vision;

    public SwerveDriveDriveToNote() {
        this.swerve = SwerveDrive.getInstance();
        this.vision = AbstractNoteVision.getInstance();

        addRequirements(swerve);
    }

    private double getDistanceToIntake() {
        return vision.getDistanceToNote() - Swerve.LENGTH / 2.0;
    }

    @Override
    public void execute() {
        if (!vision.hasNoteData()) {
            swerve.stop();
            return;
        }

        swerve.setChassisSpeeds(new ChassisSpeeds(-DRIVE_SPEED.get(), 0, 0));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(getDistanceToIntake()) < THRESHOLD_X.get();
    }
}