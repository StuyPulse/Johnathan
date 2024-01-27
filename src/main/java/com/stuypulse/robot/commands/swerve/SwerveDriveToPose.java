/************************ PROJECT JON *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import static com.stuypulse.robot.constants.Settings.Alignment.*;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.HolonomicController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class SwerveDriveToPose extends Command {
    private final SwerveDrive swerve;
    private final Pose2d targetPose;

    // Holonomic control
    private final HolonomicController controller;
    private final BStream aligned;

    private final FieldObject2d targetPose2d;

    public SwerveDriveToPose(Pose2d targetPose){
        this.swerve = SwerveDrive.getInstance();
        this.targetPose = targetPose;

        controller = new HolonomicController(
            new PIDController(Translation.P,Translation.I,Translation.D),
            new PIDController(Translation.P, Translation.I, Translation.D),
            new AnglePIDController(Rotation.P, Rotation.I, Rotation.D));

        SmartDashboard.putData("Alignment/Controller", controller);

        aligned = BStream.create(this::isAligned).filtered(new BDebounceRC.Rising(DEBOUNCE_TIME));

        targetPose2d = AbstractOdometry.getInstance().getField().getObject("Target Pose");
        addRequirements(swerve);
    }

    private boolean isAligned() {
        return controller.isDone(X_TOLERANCE.get(), Y_TOLERANCE.get(), ANGLE_TOLERANCE.get());
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Pose2d currentPose = AbstractOdometry.getInstance().getPose();

        targetPose2d.setPose(targetPose);

        controller.update(targetPose, currentPose);
        swerve.setChassisSpeeds(controller.getOutput());
    }

    @Override
    public boolean isFinished(){
        return aligned.get();
    }

    public void end(boolean interupted) {
        swerve.stop();
        targetPose2d.setPose(Double.NaN, Double.NaN, new Rotation2d(Double.NaN));
    }

}
