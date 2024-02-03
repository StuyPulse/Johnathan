package com.stuypulse.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ZeroAuton extends SequentialCommandGroup {
    public ZeroAuton() {
        addCommands(
                /** Do a whole lot of nothing */
                );
    }
}
