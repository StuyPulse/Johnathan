package com.stuypulse.robot.commands.shooter.HorizontalShooter;
 
import com.stuypulse.robot.subsystems.shooter.HorizontalShooter.HorizontalShooter;
import com.stuypulse.robot.constants.Settings;


public class HorizontalShooterPodiumShot extends HorizontalShooterSetRPM {
    
    public HorizontalShooterPodiumShot(HorizontalShooter horizontalShooter){
        super(horizontalShooter, Settings.HORIZONTAL_PODIUM_TOP_RPM, Settings.HORIZONTAL_PODIUM_BOTTOM_RPM);
    }

}