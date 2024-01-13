package com.stuypulse.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import com.stuypulse.robot.subsystems.intake.*;

public class IntakeStop extends Command {

    private Intake intake;

    public IntakeStop() {

        intake = Intake.getInstance();

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setSpeed(0);
    }

}