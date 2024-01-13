package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings.RobotType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {

    public static final Intake instance;

    static {
        if (Robot.robotType == RobotType.OFFSEASON_BOT)
            instance = new IntakeImpl();
        else
            instance = new EmptyIntake();
    }
    
    public static Intake getInstance() {
        return instance;
    }

    public abstract void setSpeed(double topSpeed, double bottomSpeed);

    public void childPeriodic() {};

    @Override
    public void periodic() {
        childPeriodic();
    }
}