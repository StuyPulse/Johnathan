package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.stuylib.control.feedback.PIDController;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import java.lang.ModuleLayer.Controller;

// import com.revrobotics.CANSparkMax;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.robot.constants.Settings;

public abstract class Shooter extends SubsystemBase {
    private static final Shooter shooter;

    static {
        shooter = new ShooterImpl();
    }

    public static Shooter getInstance() {
        return shooter;
    }

    public SmartNumber targetRPM;
    public PIDController controller; 

    public Shooter() {
        targetRPM = new SmartNumber("Shooter/Target RPM", 0.0);
        controller = new PIDController(Settings.Shooter.kP.getAsDouble(), Settings.Shooter.kI.getAsDouble(), Settings.Shooter.kD.getAsDouble());
    }
    
    public void stop() {
        targetRPM.set(0);
    }

    public void setShooterRPM(Number speed) {
        targetRPM.set(speed);
    }

    public abstract double getShooterRPM();

    public double getTargetRPM() {
        return targetRPM.get();
    }
}