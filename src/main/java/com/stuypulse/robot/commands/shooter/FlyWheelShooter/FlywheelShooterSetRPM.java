package com.stuypulse.robot.commands.shooter.FlyWheelShooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.stuypulse.robot.subsystems.shooter.FlywheelShooter.FlywheelShooter;

public class FlywheelShooterSetRPM extends InstantCommand {
    private final FlywheelShooter flywheelShooter;
    private final Number targetRPM;

    public FlywheelShooterSetRPM(FlywheelShooter flywheelShooter, Number targetRPM) {
        this.flywheelShooter = flywheelShooter;
        this.targetRPM = targetRPM;
        addRequirements(flywheelShooter);
    }

    @Override
    public void initialize() {
        flywheelShooter.setTargetRPM(targetRPM.doubleValue());
    }


}