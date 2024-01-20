package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FourNote541 extends SequentialCommandGroup{
    // public static final PathConstraints CONSTRAINTS = new PathConstraints(0, 0, 0, 0);
    // List<PathPlannerPath> path = PathPlannerAuto.getPathGroupFromAutoFile("4 Note End Speaker");
    PathPlannerAuto auto = new PathPlannerAuto("4 Note (5, 4, 1)");

    // HashMap<String, PathPlannerPath> paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
    //     path, "Intake second note", "Shoot second note", "Intake third note", "Shoot third note", "Intake fourth note", "Shoot fourth note"
    // );

    public FourNote541() {
        addCommands(
            new SwerveDriveFollowTrajectory(auto)
        );
    //     // shoot
    //     addCommands(
        
    //     );

    //     // move
    //     addCommands(
    //      new SwerveDriveFollowTrajectory(PathPlannerPath.fromPathFile("4 Note (5,4,1)")).robotRelative()
    //     );
    // }

    // public FourNote541(TwoWheelShooter shooter) {
    //     // Shoot first note
    //     addCommands(
    //         new TwoWheelShooterPodiumShot(shooter)
    //     );

    //     // Intake second note
    //     addCommands(
    //         new SwerveDriveFollowTrajectory(paths.get("Intake second note")).robotRelative(),
    //         new IntakeAcquire()
    //     );

    //     // Move + shoot second note
    //     addCommands(
    //         new SwerveDriveFollowTrajectory(paths.get("Shoot second note")).robotRelative(),
    //         new TwoWheelShooterPodiumShot(shooter)
    //     );

    //     // Intake third note
    //     addCommands(
    //         new SwerveDriveFollowTrajectory(paths.get("Intake third note")).robotRelative(),
    //         new IntakeAcquire()
    //     );

    //     // Move + shoot third note
    //     addCommands(
    //         new SwerveDriveFollowTrajectory(paths.get("Shoot third note")).robotRelative(),
    //         new TwoWheelShooterPodiumShot(shooter)
    //     );
    }
}