package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.commands.shooter.TwoWheelShooter.TwoWheelShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;
import com.stuypulse.robot.subsystems.shooter.TwoWheelShooter.TwoWheelShooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TwoNote extends SequentialCommandGroup {
    public static final PathConstraints CONSTRAINTS = new PathConstraints(0, 0, 0, 0);

    public TwoNote(TwoWheelShooter shooter) {
        // Shoot first note
        addCommands(
            new TwoWheelShooterPodiumShot(shooter)
        );
        
        // Move to + intake second note
        addCommands(
            new SwerveDriveFollowTrajectory(PathPlannerPath.fromPathFile("2 Note")).robotRelative(),
            new IntakeAcquire()
        ); 
        
        //Shoot second note
        addCommands(
            new TwoWheelShooterPodiumShot(shooter)
        );
    }
}