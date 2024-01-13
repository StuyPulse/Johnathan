package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FourNoteEndFour extends SequentialCommandGroup{
       public static final PathConstraints CONSTRAINTS = new PathConstraints(0, 0, 0 ,0);
        public FourNoteEndFour() {
       
        // Shoot
        addCommands(
            
        );

        // Move
        addCommands(
            new SwerveDriveFollowTrajectory(PathPlannerPath.fromPathFile("4 Note End 4")).robotRelative()
        );
    }   
}
