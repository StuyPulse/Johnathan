/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.stuypulse.robot.commands.leds.LEDAlign;
import com.stuypulse.robot.constants.Settings.RobotType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    public static final RobotType robotType;

    static {
        if (Robot.isSimulation())
            robotType = RobotType.SIM;
        else
            robotType = RobotType.fromString(System.getenv("serialnum"));
    }

    private RobotContainer robot;
    private Command auto;

    private CommandScheduler scheduler;

    public enum MatchState {
        AUTO,
        TELEOP,
        TEST,
        DISABLE
    }

    private static MatchState state = MatchState.DISABLE;

    /*************************/
    /*** ROBOT SCHEDULEING ***/
    /*************************/

    @Override
    public void robotInit() {
        robot = new RobotContainer();

        scheduler = CommandScheduler.getInstance();

        state = MatchState.DISABLE;
        SmartDashboard.putString("Match State", state.name());
        SmartDashboard.putString("Robot Type", robotType.name());

        Pathfinding.setPathfinder(new LocalADStar());
    }

    @Override
    public void robotPeriodic() {
        scheduler.run();
    }

    /*********************/
    /*** DISABLED MODE ***/
    /*********************/

    @Override
    public void disabledInit() {
        state = MatchState.DISABLE;
        SmartDashboard.putString("Match State", state.name()); 
    }

    @Override
    public void disabledPeriodic() {
        scheduler.schedule(new LEDAlign(new PathPlannerAuto("0 Auton")));
    }

    /***********************/
    /*** AUTONOMOUS MODE ***/
    /***********************/  

    @Override
    public void autonomousInit() {
        auto = robot.getAutonomousCommand();

        if (auto != null) {
            auto.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    /*******************/
    /*** TELEOP MODE ***/
    /*******************/

    @Override
    public void teleopInit() {
        state = MatchState.TELEOP;
        SmartDashboard.putString("Match State", state.name());
        if (auto != null) {
            auto.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    /*****************/
    /*** TEST MODE ***/
    /*****************/

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    public static MatchState getMatchState() {
        return state;
    }
}
