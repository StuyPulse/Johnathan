package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TwoNoteCenterSix extends SequentialCommandGroup {
    // public static final PathConstraints CONSTRAINTS = new PathConstraints(0, 0, 0, 0);
    // List<PathPlannerPath> path = PathPlannerAuto.getPathGroupFromAutoFile("2 Note Center 6");
    PathPlannerAuto auto = new PathPlannerAuto("2 Note Center 6");
    
    // HashMap<String, PathPlannerPath> paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
    //     path, "Intake second note", "Shoot second note"
    // );

    public TwoNoteCenterSix() {
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
    }
}