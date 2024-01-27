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
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveToNote extends Command {

    // Subsystems
    private final SwerveDrive swerve;
    private final AbstractNoteVision vision;

    // Holonomic control
    private final HolonomicController controller;
    private final BStream aligned;

    public SwerveDriveToNote(){
        this.swerve = SwerveDrive.getInstance();
        this.vision = AbstractNoteVision.getInstance();

        controller = new HolonomicController(
            new PIDController(Translation.P,Translation.I,Translation.D),
            new PIDController(Translation.P, Translation.I, Translation.D),
            new AnglePIDController(Rotation.P, Rotation.I, Rotation.D)
        );

        SmartDashboard.putData("Note Detection/Controller", controller);

        aligned = BStream.create(this::isAligned).filtered(new BDebounceRC.Rising(DEBOUNCE_TIME));

        addRequirements(swerve);
    }

    private boolean isAligned() {
        return controller.isDone(THRESHOLD_X.get(), THRESHOLD_Y.get(), THRESHOLD_ANGLE.get());
    }

    @Override
    public void execute() {
        double noteDistanceToIntake  = vision.getDistanceToNote() - Swerve.LENGTH / 2.0;
        Rotation2d rotationToNote = vision.getRotationToNote();

        Translation2d noteRelativeTranslation = new Translation2d(noteDistanceToIntake, rotationToNote);

        // origin is center of intake facing forwards
        Pose2d origin = new Pose2d(0, 0, new Rotation2d());
        Pose2d robotRelativePose = new Pose2d(noteRelativeTranslation, rotationToNote);

        if (!vision.hasNoteData()) {
            robotRelativePose = new Pose2d(origin.getTranslation(), robotRelativePose.getRotation());
        }

        swerve.setChassisSpeeds(controller.update(robotRelativePose, origin));

        SmartDashboard.putBoolean("Note Detection/Is Aligned", aligned.get());
    }

    @Override
    public boolean isFinished() {
        return aligned.get();
    }
}