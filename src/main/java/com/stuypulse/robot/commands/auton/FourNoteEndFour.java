package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;
import com.stuypulse.robot.subsystems.shooter.TwoWheelShooter.TwoWheelShooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FourNoteEndFour extends SequentialCommandGroup {
    // public static final PathConstraints CONSTRAINTS = new PathConstraints(0, 0, 0, 0);
    // List<PathPlannerPath> path = PathPlannerAuto.getPathGroupFromAutoFile("4 Note End 4");
    PathPlannerAuto auto = new PathPlannerAuto("4 Note End 4");

    // HashMap<String, PathPlannerPath> paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
    //     path, "Intake second note", "Shoot second note", "Intake third note", "Shoot third note", "Intake fourth note", "Shoot fourth note", "Move to center"
    // );

    public FourNoteEndFour() {
        addCommands(
            new SwerveDriveFollowTrajectory(auto)
        );
        
    //    // Shoot first note
    //    addCommands(
    //        new TwoWheelShooterPodiumShot(shooter)
    //    );
    //
    //    // Intake second note
    //    addCommands(
    //        new SwerveDriveFollowTrajectory(paths.get("Intake second note")).robotRelative(),
    //        new IntakeAcquire()
    //    );
    //
    //    // Move + shoot second note
    //    addCommands(
    //        new SwerveDriveFollowTrajectory(paths.get("Shoot second note")).robotRelative(),
    //        new TwoWheelShooterPodiumShot(shooter)
    //    );
    //
    //    // Intake third note
    //    addCommands(
    //        new SwerveDriveFollowTrajectory(paths.get("Intake third note")).robotRelative(),
    //        new IntakeAcquire()
    //    );
    //
    //    // Move + shoot third note
    //    addCommands(
    //        new SwerveDriveFollowTrajectory(paths.get("Shoot third note")).robotRelative(),
    //        new TwoWheelShooterPodiumShot(shooter)
    //    );

        // Intake fourth note
    //    addCommands(
    //        new SwerveDriveFollowTrajectory(paths.get("Intake fourth note")).robotRelative(),
    //        new IntakeAcquire()
    //    );

        // Move + shoot fifth note
    //    addCommands(
    //        new SwerveDriveFollowTrajectory(paths.get("Shoot fourth note")).robotRelative(),
    //        new TwoWheelShooterPodiumShot(shooter)
    //    );

        // Move to center
    //    addCommands(
    //        new SwerveDriveFollowTrajectory(paths.get("Move to center")).robotRelative()
    //    );
    }
}