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

    private final CANSparkMax topMotor;
    private final RelativeEncoder topEncoder;

    private final CANSparkMax bottomMotor;
    private final RelativeEncoder bottomEncoder;
    
    public Intake() {
        topMotor = new CANSparkMax(Ports.Intake.TOP_MOTOR, MotorType.kBrushless);
        topEncoder = topMotor.getEncoder();

        bottomMotor = new CANSparkMax(Ports.Intake.TOP_MOTOR, MotorType.kBrushless);
        bottomEncoder = topMotor.getEncoder();

    }
  
    public void setSpeed(double topSpeed, double bottomSpeed) {
        topMotor.set(topSpeed);
        bottomMotor.set(bottomSpeed);
    }

    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Top Motor Speed",topMotor.get());
        SmartDashboard.putNumber("Intake/Bottom Motor Speed", bottomMotor.get());

        SmartDashboard.putNumber("Intake/Top Motor RPM", topEncoder.getVelocity());
        SmartDashboard.putNumber("Intake/Bottom Motor RPM", bottomEncoder.getVelocity());
    }

}