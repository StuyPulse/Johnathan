package com.stuypulse.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;


public class HorizontalShooterImpl extends HorizontalShooter {
    private CANSparkMax topMotor;
    private CANSparkMax bottomMotor;
    private RelativeEncoder topEncoder;
    private RelativeEncoder bottomEncoder;

public HorizontalShooterImpl(){
    topMotor = new CANSparkMax(Ports.Shooter.HORIZONTAL_TOP_SHOOTER, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(Ports.Shooter.HORIZONTAL_BOTTOM_SHOOTER, MotorType.kBrushless);
    topEncoder = topMotor.getEncoder();
    bottomEncoder = bottomMotor.getEncoder();
    Motors.Shooter.HORIZONTAL_TOP_SHOOTER.configure(topMotor);
    Motors.Shooter.HORIZONTAL_BOTTOM_SHOOTER.configure(bottomMotor);

}
    @Override
    public double getTopMotorRPM(){
        return topEncoder.getVelocity();
    }
    
    @Override
    public double getBottomMotorRPM(){
        return bottomEncoder.getVelocity();
    }

    @Override
    public double getTopTargetRPM() {
        return topTargetRPM.get();
    }

    @Override
    public double getBottomTargetRPM() {
        return bottomTargetRPM.get();
    }

    @Override
    public void stop() {
        setTargetRPM(0,0);
    }

    @Override
    protected void setTopMotorVoltageImpl(double voltage) {
        topMotor.setVoltage(voltage);
    }

    @Override
    protected void setBottomMotorVoltageImpl (double voltage) {
        bottomMotor.setVoltage(voltage);
    }

    @Override
    public void periodicallyCalled(){
        SmartDashboard.putNumber("Top Shooter/RPM", getTopMotorRPM());
        SmartDashboard.putNumber("Bottom Shooter/RPM", getBottomMotorRPM());
        SmartDashboard.putNumber("Top Shooter/Target RPM", getTopTargetRPM());
        SmartDashboard.putNumber("Bottom Shooter/Target RPM", getBottomTargetRPM());
        SmartDashboard.putNumber("Top Shooter/Voltage", topMotor.getBusVoltage());
        SmartDashboard.putNumber("Bottom Shooter/Voltage", bottomMotor.getBusVoltage());
    }
}
