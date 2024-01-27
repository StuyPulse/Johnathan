package com.stuypulse.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.*;

public class IntakeDeacquire extends InstantCommand {

    private AbstractIntake intake;

    public IntakeDeacquire() {

        intake = AbstractIntake.getInstance();

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setSpeed(
            Settings.Intake.DEACQUIRE_SPEED_TOP.get(), 
            Settings.Intake.DEACQUIRE_SPEED_BOTTOM.get()
        );
    }

}