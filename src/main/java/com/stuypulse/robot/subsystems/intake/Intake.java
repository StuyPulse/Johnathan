package com.stuypulse.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends AbstractIntake {

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    protected Intake() {
        motor = new CANSparkMax(Ports.Intake.MOTOR, MotorType.kBrushless);
        encoder = motor.getEncoder();
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }


    @Override
    public void childPeriodic() {
        SmartDashboard.putNumber("Intake/Motor Speed", motor.get());
        SmartDashboard.putNumber("Intake/Motor RPM", encoder.getVelocity());
    }
}
