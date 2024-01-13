package com.stuypulse.robot.subsystems.shooter.FlywheelShooter;

import com.stuypulse.robot.constants.Settings.FlyWheelShooter.*;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class FlywheelShooter extends SubsystemBase {

    protected SmartNumber targetRPM;
    protected Controller controller; 

    public FlywheelShooter() {
        targetRPM = new SmartNumber("FlyWheelShooter/Target RPM", 0.0);
        controller = new MotorFeedforward(FeedForward.kS.getAsDouble(), FeedForward.kV.getAsDouble(), FeedForward.kA.getAsDouble()).velocity()
                    .add(new PIDController(PID.kP.getAsDouble(), PID.kI.getAsDouble(), PID.kD.getAsDouble()));
    }
    
    public abstract void stop();

    public abstract double getShooterRPM();

    public void setTargetRPM(double rpm) {
        targetRPM.set(rpm);
    }

    protected abstract void setShooterVoltageImpl(double voltage);

    public double getTargetRPM() {
        return targetRPM.getAsDouble();
    }

    @Override
    public final void periodic() {
        controller.update(getTargetRPM(), getShooterRPM());
        setShooterVoltageImpl(controller.getOutput());
        periodicallyCalled();
    }
    public void periodicallyCalled() {}
}