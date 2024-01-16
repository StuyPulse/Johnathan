package com.stuypulse.robot.commands.shooter.TwoWheelShooter;

import com.stuypulse.robot.subsystems.shooter.TwoWheelShooter.TwoWheelShooter;

import com.stuypulse.robot.constants.Settings.*; 

public class TwoWheelShooterPodiumShot extends TwoWheelShooterSetRPM {
    public TwoWheelShooterPodiumShot(TwoWheelShooter twoWheelShooter){
        super(twoWheelShooter, Shooter.TWO_WHEEL_PODIUM_LEFT_RPM, Shooter.TWO_WHEEL_PODIUM_LEFT_RPM);

    }
}