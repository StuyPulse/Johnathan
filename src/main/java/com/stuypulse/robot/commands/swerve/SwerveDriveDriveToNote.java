/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.swerve;

import static com.stuypulse.robot.constants.Settings.NoteDetection.*;

import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.notevision.AbstractNoteVision;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDriveToNote extends Command {

    // Subsystems
    private final SwerveDrive swerve;
    private final AbstractOdometry odometry;
    private final AbstractNoteVision vision;

    // Holonomic control
    private final HolonomicController controller;
    private final BStream aligned;

    public SwerveDriveDriveToNote(){
        this.swerve = SwerveDrive.getInstance();
        this.odometry = AbstractOdometry.getInstance();
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
        Translation2d targetTranslation = odometry.getTranslation().plus(
            new Translation2d(Units.inchesToMeters(18), 0).rotateBy(odometry.getRotation()));

        Rotation2d targetRotation = vision.getEstimatedNotePose().minus(targetTranslation).getAngle();

        Pose2d targetPose = new Pose2d(targetTranslation, targetRotation);

        if (!vision.hasNoteData()) {
            swerve.setChassisSpeeds(controller.update(targetPose, new Pose2d(targetTranslation, odometry.getRotation())));
        }

        swerve.setChassisSpeeds(controller.update(targetPose, odometry.getPose()));

        SmartDashboard.putBoolean("Note Detection/Is Aligned", aligned.get());
    }

    @Override
    public boolean isFinished() {
        return aligned.get();
    }
}