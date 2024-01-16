/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.commands.shooter.HorizontalShooter.HorizontalShooterPodiumShot;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.robot.subsystems.shooter.FlywheelShooter.FlywheelShooter;
import com.stuypulse.robot.subsystems.shooter.FlywheelShooter.FlywheelShooterImpl;

import com.stuypulse.robot.subsystems.shooter.TwoWheelShooter.TwoWheelShooter;
import com.stuypulse.robot.subsystems.shooter.TwoWheelShooter.TwoWheelShooterImpl;

import com.stuypulse.robot.subsystems.shooter.HorizontalShooter.HorizontalShooter;
import com.stuypulse.robot.subsystems.shooter.HorizontalShooter.HorizontalShooterImpl;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystems
    // public final FlywheelShooter shooter = new FlywheelShooterImpl();
    // public final TwoWheelShooter shooter = new TwoWheelShooterImpl();
    public final HorizontalShooter shooter = new HorizontalShooterImpl();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container
    public RobotContainer() {
        // Named commands
        NamedCommands.registerCommand("Run intake", new IntakeAcquire());
        NamedCommands.registerCommand("Shoot note", new HorizontalShooterPodiumShot(shooter));

        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {}

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {}

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
