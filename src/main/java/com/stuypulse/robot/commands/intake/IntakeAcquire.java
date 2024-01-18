package com.stuypulse.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.*;

public class IntakeAcquire extends Command {

    private AbstractIntake intake;

    public IntakeAcquire() {

        intake = AbstractIntake.getInstance();

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setSpeed(
            Settings.Intake.ACQUIRE_SPEED.get()
        );
    }

}