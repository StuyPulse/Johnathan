package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Mobility extends SequentialCommandGroup {
    // public static final PathConstraints CONSTRAINTS = new PathConstraints(0, 0, 0, 0);
    PathPlannerAuto auto = new PathPlannerAuto("Mobility");


    public Mobility() {
        addCommands(
            new SwerveDriveFollowTrajectory(auto)
        );
        // addCommands(
        //     new SwerveDriveFollowTrajectory(PathPlannerPath.fromPathFile("Mobility")).robotRelative()
        // );    
    }
}

