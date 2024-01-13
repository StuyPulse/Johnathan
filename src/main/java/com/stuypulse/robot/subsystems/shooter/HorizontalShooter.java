package com.stuypulse.robot.subsystems.shooter;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.network.SmartNumber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.robot.constants.Settings.HorizontalShooter.*;


public abstract class HorizontalShooter extends SubsystemBase {
    protected SmartNumber topTargetRPM;
    protected SmartNumber bottomTargetRPM;
    protected Controller topController;
    protected Controller bottomController;

    public HorizontalShooter(){
        topTargetRPM = new SmartNumber("HorizontalShooter/Top Target RPM", 0.0);
        bottomTargetRPM = new SmartNumber("HorizontalShooter/Bottom Target RPM", 0.0);
        topController = new MotorFeedforward(FeedForward.kS.getAsDouble(), FeedForward.kV.getAsDouble(), FeedForward.kA.getAsDouble()).velocity()
                            .add(new PIDController(PID.kP.getAsDouble(),PID.kI.getAsDouble(), PID.kD.getAsDouble()));
        bottomController = new MotorFeedforward(FeedForward.kS.getAsDouble(), FeedForward.kV.getAsDouble(), FeedForward.kA.getAsDouble()).velocity()
                            .add(new PIDController(PID.kP.getAsDouble(),PID.kI.getAsDouble(), PID.kD.getAsDouble()));
    }

    public abstract void stop();

    public double getTopTargetRPM() {
        return topTargetRPM.getAsDouble();
    }

    public double getBottomTargetRPM() {
        return bottomTargetRPM.getAsDouble();
    }
    
    public void setTargetRPM(Number topTargetRPM, Number bottomTargetRPM) {
        this.topTargetRPM.set(topTargetRPM.doubleValue());
        this.bottomTargetRPM.set(bottomTargetRPM.doubleValue());
    }

    public abstract double getTopMotorRPM(); 
    public abstract double getBottomMotorRPM(); 

    protected abstract void setTopMotorVoltageImpl(double voltage);
    protected abstract void setBottomMotorVoltageImpl(double voltage);

    @Override
    public final void periodic(){
        topController.update(getTopTargetRPM(), getTopMotorRPM());
        bottomController.update(getBottomTargetRPM(), getBottomMotorRPM());
        setTopMotorVoltageImpl(topController.getOutput());
        setBottomMotorVoltageImpl(bottomController.getOutput());
        periodicallyCalled();
    }
    public void periodicallyCalled() {}

}
