package com.stuypulse.robot.commands.auton;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.commands.shooter.TwoWheelShooter.TwoWheelShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;
import com.stuypulse.robot.subsystems.shooter.TwoWheelShooter.TwoWheelShooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ThreeNoteCenter76 extends SequentialCommandGroup {
    public static final PathConstraints CONSTRAINTS = new PathConstraints(0, 0, 0, 0);
    List<PathPlannerPath> path = PathPlannerAuto.getPathGroupFromAutoFile("3 Note Center (7, 6)");

    HashMap<String, PathPlannerPath> paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
        path, "Intake second note", "Shoot second note", "Intake third note", "Shoot third note"
    );

    public ThreeNoteCenter76(TwoWheelShooter shooter) {
        // Shoot first note
        addCommands(
            new TwoWheelShooterPodiumShot(shooter)
        );

        // Intake second note
        addCommands(
            new SwerveDriveFollowTrajectory(paths.get("Intake second note")).robotRelative(),
            new IntakeAcquire()
        );

        // Move + shoot second note
        addCommands(
            new SwerveDriveFollowTrajectory(paths.get("Shoot second note")).robotRelative(),
            new TwoWheelShooterPodiumShot(shooter)
        );

        // Intake third note
        addCommands(
            new SwerveDriveFollowTrajectory(paths.get("Intake third note")).robotRelative(),
            new IntakeAcquire()
        );

        // Move + shoot third note
        addCommands(
            new SwerveDriveFollowTrajectory(paths.get("Shoot third note")).robotRelative(),
            new TwoWheelShooterPodiumShot(shooter)
        );
    }
}