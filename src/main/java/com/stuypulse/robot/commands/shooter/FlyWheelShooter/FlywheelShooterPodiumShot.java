package com.stuypulse.robot.commands.shooter.FlyWheelShooter;
import com.stuypulse.robot.constants.Settings.*;
import com.stuypulse.robot.subsystems.shooter.FlywheelShooter.FlywheelShooter;

public class FlywheelShooterPodiumShot extends FlywheelShooterSetRPM{
    public FlywheelShooterPodiumShot(FlywheelShooter shooter) {
        super(shooter, Shooter.FLYWHEEL_PODIUM_RPM);
    }
}
