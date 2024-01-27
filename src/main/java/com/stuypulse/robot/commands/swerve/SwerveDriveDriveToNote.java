/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.swerve;

import static com.stuypulse.robot.constants.Settings.NoteDetection.*;

import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.NoteDetection;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.notevision.AbstractNoteVision;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDriveToNote extends Command {

    private final SwerveDrive swerve;
    private final AbstractNoteVision vision;

    private final Controller distanceController;

    public SwerveDriveDriveToNote() {
        this.swerve = SwerveDrive.getInstance();
        this.vision = AbstractNoteVision.getInstance();

        distanceController = new PIDController(Alignment.Translation.P, Alignment.Translation.I, Alignment.Translation.D);

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

        swerve.setChassisSpeeds(new ChassisSpeeds(distanceController.update(AbstractNoteVision.getInstance().getDistanceToNote(), 0), 0, 0));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(getDistanceToIntake()) < THRESHOLD_X.get();
    }
}