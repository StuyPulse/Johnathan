/************************ PROJECT JIM *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.swerve;

import static com.stuypulse.robot.constants.Settings.NoteDetection.*;

import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.vision.NoteVision;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDriveToNote extends Command {

    private static final double CUTOFF_DISTANCE = Units.inchesToMeters(25);

    // Subsystems
    private final SwerveDrive swerve;
    private final AbstractOdometry odometry;
    private final NoteVision vision;

    private final HolonomicController controller;

    public SwerveDriveDriveToNote() {
        this.swerve = SwerveDrive.getInstance();
        this.odometry = AbstractOdometry.getInstance();
        this.vision = NoteVision.getInstance();

        controller = new HolonomicController(
            new PIDController(Translation.P,Translation.I,Translation.D),
            new PIDController(Translation.P, Translation.I, Translation.D),
            new AnglePIDController(Rotation.P, Rotation.I, Rotation.D)
        );

        SmartDashboard.putData("Note Detection/Controller", controller);

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        Translation2d targetTranslation = odometry.getTranslation().plus(
            new Translation2d(Swerve.CENTER_TO_INTAKE_FRONT, 0).rotateBy(odometry.getRotation()));

        Rotation2d targetRotation = vision.getEstimatedNotePose().minus(targetTranslation).getAngle();

        Pose2d targetPose = new Pose2d(targetTranslation, targetRotation);

        if (vision.hasNoteData()) {
            // drive to note
            swerve.setChassisSpeeds(controller.update(targetPose, odometry.getPose()));
        }
        else {
            // only rotate toward saved note pose
            swerve.setChassisSpeeds(controller.update(targetPose, new Pose2d(targetTranslation, odometry.getRotation())));
        }

        SmartDashboard.putNumber("Note Detection/Target Rotation", targetPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Note Detection/X", targetPose.getX());
        SmartDashboard.putNumber("Note Detection/Y", targetPose.getY());
        SmartDashboard.putNumber("Note Detection/D", vision.getEstimatedNotePose().minus(odometry.getTranslation()).getNorm());
    }

    @Override
    public boolean isFinished() {
        return vision.getEstimatedNotePose().minus(odometry.getTranslation()).getNorm() < CUTOFF_DISTANCE;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}