package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OneNoteMobility extends SequentialCommandGroup {
    // public static final PathConstraints CONSTRAINTS = new PathConstraints(0, 0, 0, 0);
    PathPlannerAuto auto = new PathPlannerAuto("1 Note + Mobility");

    public OneNoteMobility() {
        addCommands(
            new SwerveDriveFollowTrajectory(auto)
        );
        // // Move
        // addCommands(
        //     new SwerveDriveFollowTrajectory(PathPlannerPath.fromPathFile("1 Note + Mobility")).robotRelative()
        // );

        // //shoot
        // addCommands(
        //     new TwoWheelShooterPodiumShot(shooter)
        // );
    }   
}
