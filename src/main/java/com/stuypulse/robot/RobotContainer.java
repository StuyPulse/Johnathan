/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveToNote;
import com.stuypulse.robot.commands.swerve.SwerveDriveNoteAlignedDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveToScore;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetHeading;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.commands.swerve.SwerveDriveToScore;
import com.stuypulse.robot.commands.swerve.SwerveDriveWithAiming;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.AbstractVision;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.commands.intake.IntakeDeacquire;
import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.intake.*;
import com.stuypulse.robot.subsystems.notevision.AbstractNoteVision;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);

    // Subsystems
    public final SwerveDrive swerve = SwerveDrive.getInstance();
    public final AbstractOdometry odometry = AbstractOdometry.getInstance();
    public final AbstractVision vision = AbstractVision.getInstance();
    public final AbstractIntake intake = AbstractIntake.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser;

    // Robot container

    public RobotContainer() {
        swerve.configureAutoBuilder();
        configureDefaultCommands();
        configureButtonBindings();
        configureNamedCommands();
        configureAutons();
    }

    /**********************/
    /*** NAMED COMMANDS ***/
    /**********************/

    private void configureNamedCommands() {
        NamedCommands.registerCommand("intake acquire", new IntakeAcquire());
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        driver.getDPadUp().onTrue(new SwerveDriveResetHeading(Rotation2d.fromDegrees(0)));
        driver.getDPadDown().onTrue(new SwerveDriveResetHeading(Rotation2d.fromDegrees(180)));
        driver.getDPadLeft().onTrue(new SwerveDriveResetHeading(Rotation2d.fromDegrees(90)));
        driver.getDPadRight().onTrue(new SwerveDriveResetHeading(Rotation2d.fromDegrees(270)));
        
        driver.getRightTriggerButton()
            .onTrue(new IntakeAcquire())
            .onFalse(new IntakeStop());

        driver.getLeftTriggerButton()
            .onFalse(new IntakeDeacquire())
            .onFalse(new IntakeStop());

        driver.getBottomButton().whileTrue(new SwerveDriveToPose(new Pose2d(2, 5.5, new Rotation2d(Units.degreesToRadians(180)))));
        // driver.getRightButton().whileTrue(new SwerveDriveToPose(new Pose2d(1.5, 5.5, new Rotation2d())));

        driver.getTopButton().whileTrue(new SwerveDriveWithAiming(Field.getFiducial(7).getPose().toPose2d(), driver));
        driver.getBottomButton().whileTrue(new SwerveDriveWithAiming(Field.getFiducial(8).getPose().toPose2d(), driver));

        driver.getRightBumper()
            .whileTrue(new SwerveDriveNoteAlignedDrive(driver));

        driver.getLeftBumper()
            .whileTrue(new SwerveDriveDriveToNote())
            .whileTrue(new IntakeAcquire())
            .onFalse(new IntakeStop());
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
