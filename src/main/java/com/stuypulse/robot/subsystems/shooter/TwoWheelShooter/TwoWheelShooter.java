package com.stuypulse.robot.subsystems.shooter.TwoWheelShooter;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.network.SmartNumber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.stuypulse.robot.constants.Settings.Shooter.TwoWheelShooter.FeedForward;
import com.stuypulse.robot.constants.Settings.Shooter.TwoWheelShooter.PID;

public abstract class TwoWheelShooter extends SubsystemBase {
   
    protected SmartNumber leftTargetRPM;
    protected SmartNumber rightTargetRPM;
    protected Controller leftController; 
    protected Controller rightController;
    
    public TwoWheelShooter() {
        leftTargetRPM = new SmartNumber("TwoWheelShooter/Left Target RPM", 0.0);
        rightTargetRPM = new SmartNumber("TwoWheelShooter/Right Target RPM", 0.0);
        leftController = new MotorFeedforward(FeedForward.kS.getAsDouble(), FeedForward.kV.getAsDouble(), FeedForward.kA.getAsDouble()).velocity()
                            .add(new PIDController(PID.kP.getAsDouble(),PID.kI.getAsDouble(), PID.kD.getAsDouble()));
        rightController = new MotorFeedforward(FeedForward.kS.getAsDouble(), FeedForward.kV.getAsDouble(), FeedForward.kA.getAsDouble()).velocity()
                            .add(new PIDController(PID.kP.getAsDouble(),PID.kI.getAsDouble(), PID.kD.getAsDouble()));
    }
    
    public abstract void stop();

    public double getLeftTargetRPM() {
        return leftTargetRPM.getAsDouble();
    }

    public double getRightTargetRPM() {
        return rightTargetRPM.getAsDouble();
    }

    public void setTargetRPM(Number leftTargetRPM, Number rightTargetRPM) {
        this.leftTargetRPM.set(leftTargetRPM.doubleValue());
        this.rightTargetRPM.set(rightTargetRPM.doubleValue());
    }

    public abstract double getLeftShooterRPM(); 
    public abstract double getRightShooterRPM(); 

    protected abstract void setLeftMotorVoltageImpl(double voltage);
    protected abstract void setRightMotorVoltageImpl(double voltage);

    @Override
    public final void periodic(){
        leftController.update(getLeftTargetRPM(), getLeftShooterRPM());
        rightController.update(getRightTargetRPM(), getRightShooterRPM());
        setLeftMotorVoltageImpl(leftController.getOutput());
        setRightMotorVoltageImpl(rightController.getOutput());
        periodicallyCalled();
    }
    public void periodicallyCalled() {}

}