package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.shooter.TwoWheelShooter.TwoWheelShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;
import com.stuypulse.robot.subsystems.shooter.TwoWheelShooter.TwoWheelShooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OneNoteMobility extends SequentialCommandGroup {
    public static final PathConstraints CONSTRAINTS = new PathConstraints(0, 0, 0, 0);

    public OneNoteMobility(TwoWheelShooter shooter) {

        // Move
        addCommands(
            new SwerveDriveFollowTrajectory(PathPlannerPath.fromPathFile("1 Note + Mobility")).robotRelative()
        );

        //shoot
        addCommands(
            new TwoWheelShooterPodiumShot(shooter)
        );
    }   
}
