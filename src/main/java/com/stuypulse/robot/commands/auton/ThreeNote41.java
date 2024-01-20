package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ThreeNote41 extends SequentialCommandGroup {
    // public static final PathConstraints CONSTRAINTS = new PathConstraints(0, 0, 0, 0);
    // List<PathPlannerPath> path = PathPlannerAuto.getPathGroupFromAutoFile("3 Note (4, 1)");
    PathPlannerAuto auto = new PathPlannerAuto("3 Note (4, 1)");

    // HashMap<String, PathPlannerPath> paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
    //     path, "Intake second note", "Shoot second note", "Intake third note", "Shoot third note"
    // );

    public ThreeNote41() {
        addCommands(
            new SwerveDriveFollowTrajectory(auto)
        );
        // // Shoot first note
        // addCommands(
        //     new TwoWheelShooterPodiumShot(shooter)
        // );

        // // Intake second note
        // addCommands(
        //     new SwerveDriveFollowTrajectory(paths.get("Intake second note")).robotRelative(),
        //     new IntakeAcquire()
        // );

        // // Move + shoot second note
        // addCommands(
        //     new SwerveDriveFollowTrajectory(paths.get("Shoot second note")).robotRelative(),
        //     new TwoWheelShooterPodiumShot(shooter)
        // );

        // // Intake third note
        // addCommands(
        //     new SwerveDriveFollowTrajectory(paths.get("Intake third note")).robotRelative(),
        //     new IntakeAcquire()
        // );

        // // Move + shoot third note
        // addCommands(
        //     new SwerveDriveFollowTrajectory(paths.get("Shoot third note")).robotRelative(),
        //     new TwoWheelShooterPodiumShot(shooter)
        // );
    }
}