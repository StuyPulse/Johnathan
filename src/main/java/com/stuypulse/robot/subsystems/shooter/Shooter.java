package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.robot.constants.Settings.Shooter.*;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {
    private static final Shooter shooter;

    static {
        shooter = new ShooterImpl();
    }

    public static Shooter getInstance() {
        return shooter;
    }

    protected SmartNumber targetRPM;
    protected Controller controller; 

    public Shooter() {
        targetRPM = new SmartNumber("Shooter/Target RPM", 0.0);
        controller = new MotorFeedforward(FeedForward.kS.getAsDouble(), FeedForward.kV.getAsDouble(), FeedForward.kA.getAsDouble()).velocity()
                    .add(new PIDController(PID.kP.getAsDouble(), PID.kI.getAsDouble(), PID.kD.getAsDouble()));
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