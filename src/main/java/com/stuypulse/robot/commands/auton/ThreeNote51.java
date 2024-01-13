package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ThreeNote51 extends SequentialCommandGroup{
    public static final PathConstraints CONSTRAINTS = new PathConstraints(0, 0, 0, 0);

    public ThreeNote51() {
        // shoot
        addCommands(
            
        );

        //move 
        addCommands(
            new SwerveDriveFollowTrajectory(PathPlannerPath.fromPathFile("3 Note + Mobility (5,1)")).robotRelative()
        );
    }
}



    
