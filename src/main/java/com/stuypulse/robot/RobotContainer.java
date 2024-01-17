/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetHeading;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.AbstractVision;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.commands.intake.IntakeDeacquire;
import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.intake.*;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.math.geometry.Rotation2d;
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
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();

        swerve.configureAutoBuilder();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
        intake.setDefaultCommand(new IntakeStop());
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        driver.getDPadUp().onTrue(new SwerveDriveResetHeading(Rotation2d.fromDegrees(180)));
        driver.getDPadDown().onTrue(new SwerveDriveResetHeading(Rotation2d.fromDegrees(0)));
        driver.getDPadLeft().onTrue(new SwerveDriveResetHeading(Rotation2d.fromDegrees(270)));
        driver.getDPadRight().onTrue(new SwerveDriveResetHeading(Rotation2d.fromDegrees(90)));
        
        operator.getRightTriggerButton()
            .whileTrue(new IntakeAcquire())
            .onFalse(new IntakeStop());

        operator.getLeftTriggerButton()
            .whileTrue(new IntakeDeacquire())
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
