package com.stuypulse.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    public static final Intake instance;

    static {
        instance = new Intake();
    }
    
    public static Intake getInstance(){
        return instance;
    }

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    
    public Intake() {
        motor = new CANSparkMax(Ports.Intake.MOTOR, MotorType.kBrushless);
        encoder = motor.getEncoder();
    }
  
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public double getRPM() {
        return encoder.getVelocity();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Speed", motor.get());
        SmartDashboard.putNumber("Intake/RPM", getRPM());
    }

}