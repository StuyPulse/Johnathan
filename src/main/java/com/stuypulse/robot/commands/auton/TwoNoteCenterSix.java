package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TwoNoteCenterSix extends SequentialCommandGroup{
    public static final PathConstraints CONSTRAINTS = new PathConstraints(0,0,0,0);

    public TwoNoteCenterSix() {
        //shoot first note 
        addCommands(

        );

        //move
        addCommands(
            new SwerveDriveFollowTrajectory(PathPlannerPath.fromPathFile("2 Note Center 6")).robotRelative()
        );
    }
}