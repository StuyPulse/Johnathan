package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SixNote extends SequentialCommandGroup {
    PathPlannerAuto auto = new PathPlannerAuto("Six Note");

    public SixNote() {
        addCommands(
            new SwerveDriveFollowTrajectory(auto)
        );
    }
}
