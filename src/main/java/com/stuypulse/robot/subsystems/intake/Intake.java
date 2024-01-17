package com.stuypulse.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends AbstractIntake {
    private final CANSparkMax topMotor;
    private final RelativeEncoder topEncoder;

    private final CANSparkMax bottomMotor;
    private final RelativeEncoder bottomEncoder;

    protected Intake() {
        topMotor = new CANSparkMax(Ports.Intake.TOP_MOTOR, MotorType.kBrushless);
        topEncoder = topMotor.getEncoder();

        bottomMotor = new CANSparkMax(Ports.Intake.BOTTOM_MOTOR, MotorType.kBrushless);
        bottomEncoder = bottomMotor.getEncoder();
    }

    @Override
    public void setSpeed(double topSpeed, double bottomSpeed) {
        topMotor.set(topSpeed);
        bottomMotor.set(bottomSpeed);
    }


    @Override
    public void childPeriodic() {
        SmartDashboard.putNumber("Intake/Top Motor Speed",topMotor.get());
        SmartDashboard.putNumber("Intake/Bottom Motor Speed", bottomMotor.get());

        SmartDashboard.putNumber("Intake/Top Motor RPM", topEncoder.getVelocity());
        SmartDashboard.putNumber("Intake/Bottom Motor RPM", bottomEncoder.getVelocity());
    }
}
