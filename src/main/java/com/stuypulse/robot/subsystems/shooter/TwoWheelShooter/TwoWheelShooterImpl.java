package com.stuypulse.robot.subsystems.shooter.TwoWheelShooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TwoWheelShooterImpl extends TwoWheelShooter {
    private CANSparkMax leftWheel;
    private CANSparkMax rightWheel;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    public TwoWheelShooterImpl() {
        leftWheel = new CANSparkMax(Ports.Shooter.TWO_WHEEL_LEFT_MOTOR, MotorType.kBrushless);
        rightWheel = new CANSparkMax(Ports.Shooter.TWO_WHEEL_RIGHT_MOTOR, MotorType.kBrushless);
        leftEncoder = leftWheel.getEncoder();
        rightEncoder = rightWheel.getEncoder();
        Motors.Shooter.TWO_WHEEL_LEFT_SHOOTER.configure(leftWheel);
        Motors.Shooter.TWO_WHEEL_RIGHT_SHOOTER.configure(rightWheel);
    }

    @Override
    public double getLeftShooterRPM() {
        return leftEncoder.getVelocity();
    }

    @Override
    public double getRightShooterRPM() {
        return rightEncoder.getVelocity();
    }

    @Override
    public double getRightTargetRPM() {
        return rightTargetRPM.get();
    }

    @Override
    public double getLeftTargetRPM() {
        return leftTargetRPM.get();
    }
    
    @Override
    public void stop() {
        setTargetRPM(0,0);
    }

    @Override
    protected void setLeftMotorVoltageImpl(double voltage) {
        leftWheel.setVoltage(voltage);
    }

    @Override
    protected void setRightMotorVoltageImpl (double voltage) {
        rightWheel.setVoltage(voltage);
    }
    
    @Override
    public void periodicallyCalled() {
 
        SmartDashboard.putNumber("Left Shooter/RPM", getLeftShooterRPM());
        SmartDashboard.putNumber("Right Shooter/RPM", getRightShooterRPM());
        SmartDashboard.putNumber("Left Shooter/Target RPM", getLeftTargetRPM());
        SmartDashboard.putNumber("Right Shooter/Target RPM", getRightShooterRPM());
        SmartDashboard.putNumber("Left Shooter/Voltage", leftWheel.getBusVoltage());
        SmartDashboard.putNumber("Right Shooter/Voltage", rightWheel.getBusVoltage());
        

    }
}