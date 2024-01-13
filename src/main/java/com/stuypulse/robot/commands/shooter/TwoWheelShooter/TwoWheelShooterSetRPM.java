package com.stuypulse.robot.commands.shooter.TwoWheelShooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.stuypulse.robot.subsystems.shooter.TwoWheelShooter.TwoWheelShooter;

public class TwoWheelShooterSetRPM extends InstantCommand {
    private final TwoWheelShooter twoWheelShooter;
    private final Number leftTargetRPM;
    private final Number rightTargetRPM;

    public TwoWheelShooterSetRPM(TwoWheelShooter twoWheelShooter, Number leftTargetRPM, Number rightTargetRPM) {
        this.twoWheelShooter = twoWheelShooter;
        this.leftTargetRPM = leftTargetRPM;
        this.rightTargetRPM = rightTargetRPM;
        addRequirements(twoWheelShooter);
    }

    @Override
    public void initialize() {
        twoWheelShooter.setTargetRPM(leftTargetRPM.doubleValue(), rightTargetRPM.doubleValue());
    }
    
}
