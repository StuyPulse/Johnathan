package com.stuypulse.robot.commands.shooter.HorizontalShooter;
 
import com.stuypulse.robot.subsystems.shooter.HorizontalShooter.HorizontalShooter;
import com.stuypulse.robot.constants.Settings.*;

public class HorizontalShooterPodiumShot extends HorizontalShooterSetRPM {
    public HorizontalShooterPodiumShot(HorizontalShooter horizontalShooter){
        super(horizontalShooter, Shooter.HORIZONTAL_PODIUM_TOP_RPM, Shooter.HORIZONTAL_PODIUM_BOTTOM_RPM);
    }
}