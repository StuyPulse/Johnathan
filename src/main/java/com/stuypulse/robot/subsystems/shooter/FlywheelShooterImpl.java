package com.stuypulse.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FlywheelShooterImpl extends FlywheelShooter {
    private CANSparkMax flywheel;
    private RelativeEncoder encoder;

    public FlywheelShooterImpl() {
        flywheel = new CANSparkMax(Ports.Shooter.FLYWHEEL_MOTOR, MotorType.kBrushless);
        encoder = flywheel.getEncoder();
        Motors.Shooter.FLYWHEEL_SHOOTER.configure(flywheel);
    }


    @Override 
    public void stop() {
        targetRPM.set(0.0);
    }

    @Override
    public double getShooterRPM() {
        return encoder.getVelocity();
    }

    public void periodic() {
        SmartDashboard.putNumber("FlyWheel Shooter/RPM", getShooterRPM());
        SmartDashboard.putNumber("Flywheel Shooter/Target RPM", getTargetRPM());
        SmartDashboard.putNumber("Flywheel Shooter/Voltage", flywheel.getBusVoltage());

        controller.update(getTargetRPM(), getShooterRPM());
        flywheel.setVoltage(controller.getOutput());
    }
}