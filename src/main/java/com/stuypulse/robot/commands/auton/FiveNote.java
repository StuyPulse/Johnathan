package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FiveNote extends SequentialCommandGroup {
    PathPlannerAuto auto = new PathPlannerAuto("Five Note");

    public FiveNote() {
        addCommands(
            new SwerveDriveFollowTrajectory(auto)
        );
    }
}
