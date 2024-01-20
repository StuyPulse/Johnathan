package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ThreeNoteCenter78 extends SequentialCommandGroup {
    // public static final PathConstraints CONSTRAINTS = new PathConstraints(0, 0, 0, 0);
    // List<PathPlannerPath> path = PathPlannerAuto.getPathGroupFromAutoFile("3 Note Center (7, 8)");
    PathPlannerAuto auto = new PathPlannerAuto("3 Note Center (7, 8)");

    // HashMap<String, PathPlannerPath> paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
    //     path, "Intake second note", "Shoot second note", "Intake third note", "Shoot third note"
    // );

    public ThreeNoteCenter78() {
        addCommands(
            new SwerveDriveFollowTrajectory(auto)
        );
    }   
}