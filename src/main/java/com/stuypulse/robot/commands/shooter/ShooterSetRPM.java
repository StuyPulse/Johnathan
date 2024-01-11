package com.stuypulse.robot.commands.shooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.stuypulse.robot.subsystems.shooter.ShooterImpl;

public class ShooterSetRPM extends InstantCommand {
    private final ShooterImpl shooter;
    private final Number targetRPM;

    public ShooterSetRPM(ShooterImpl shooter, Number targetRPM) {
        this.shooter = shooter;
        this.targetRPM = targetRPM;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterRPM(targetRPM);
    }


}