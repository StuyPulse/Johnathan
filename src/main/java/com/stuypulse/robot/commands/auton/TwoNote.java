package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TwoNote extends SequentialCommandGroup {
    // public static final PathConstraints CONSTRAINTS = new PathConstraints(0, 0, 0, 0);
    PathPlannerAuto auto = new PathPlannerAuto("2 Note");

    public TwoNote() {
        addCommands(
            new SwerveDriveFollowTrajectory(auto)
        );
        // // Shoot first note
        // addCommands(
        //     new TwoWheelShooterPodiumShot(shooter)
        // );
        
        // // Move to + intake second note
        // addCommands(
        //     new SwerveDriveFollowTrajectory(PathPlannerPath.fromPathFile("2 Note")).robotRelative(),
        //     new IntakeAcquire()
        // ); 
        
        // //Shoot second note
        // addCommands(
        //     new TwoWheelShooterPodiumShot(shooter)
        // );
    }
}