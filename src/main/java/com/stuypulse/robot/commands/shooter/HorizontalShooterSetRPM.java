package com.stuypulse.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.stuypulse.robot.subsystems.shooter.HorizontalShooter;

public class HorizontalShooterSetRPM extends InstantCommand {
    private final HorizontalShooter horizontalShooter;
    private final Number topTargetRPM;
    private final Number bottomTargetRPM;

    public HorizontalShooterSetRPM(HorizontalShooter horizontalShooter, Number topTargetRPM, Number bottomTargetRPM) {
        this.horizontalShooter = horizontalShooter;
        this.topTargetRPM = topTargetRPM;
        this.bottomTargetRPM = bottomTargetRPM;
        addRequirements(horizontalShooter);
    }

    @Override
    public void initialize() {
        horizontalShooter.setTargetRPM(topTargetRPM, bottomTargetRPM);
    }


}

