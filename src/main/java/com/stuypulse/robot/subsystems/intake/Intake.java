package com.stuypulse.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.robot.constants.Settings.Intake.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private CANSparkMax topMotor;
    private RelativeEncoder topEncoder;

    private CANSparkMax bottomMotor;
    private RelativeEncoder bottomEncoder;


    public static Intake instance;
    
    public static Intake getInstance(){
        if(instance != null){
            return instance;
        }
        instance = new Intake();
        return instance;
    }
    
    public Intake() {

        topMotor = new CANSparkMax(Ports.topMotor , MotorType.kBrushless);
        topEncoder = topMotor.getEncoder();
        
        bottomMotor = new CANSparkMax(Ports.bottomMotor, MotorType.kBrushless);
        bottomEncoder = bottomMotor.getEncoder();
    }
  
    public void setRPM(double velocity) {
        topMotor.set(velocity);
        bottomMotor.set(-velocity);
    }

    public double getRPM() {
        return bottomEncoder.getVelocity();
    }

    @Override
    public void periodic() {
        
       SmartDashboard.putNumber("Velocity", getRPM());
       
    }

}