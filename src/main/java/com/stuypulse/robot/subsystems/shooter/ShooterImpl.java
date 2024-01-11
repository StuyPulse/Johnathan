package com.stuypulse.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterImpl extends Shooter {
    private CANSparkMax flywheel;
    private RelativeEncoder encoder;

    public ShooterImpl() {
        flywheel = new CANSparkMax(Ports.Shooter.MOTOR, MotorType.kBrushless);
        encoder = flywheel.getEncoder();
        
        //configure motors 
        Motors.SHOOTER.configure(flywheel); 
    }
    
    public double getShooterRPM() { 
        return encoder.getVelocity(); 
    }

    public void periodic() {
        SmartDashboard.putNumber("Shooter/RPM", getShooterRPM());
        SmartDashboard.putNumber("Shooter/Target RPM", getTargetRPM());
        SmartDashboard.putNumber("Shooter/Voltage", flywheel.getOutputCurrent());

        controller.update(targetRPM.getAsDouble(), getShooterRPM());
        flywheel.setVoltage(controller.getOutput());
    }
}