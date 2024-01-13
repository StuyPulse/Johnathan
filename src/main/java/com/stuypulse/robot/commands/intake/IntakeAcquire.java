package com.stuypulse.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.*;

public class IntakeAcquire extends Command {

    private Intake intake;

    public IntakeAcquire() {

        intake = Intake.getInstance();

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setSpeed(
            Settings.Intake.ACQUIRE_SPEED_TOP.get(),
            Settings.Intake.ACQUIRE_SPEED_BOTTOM.get()
        );
    }

}