package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings.RobotType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractIntake extends SubsystemBase {

    public static final AbstractIntake instance;

    static {
        if (Robot.robotType == RobotType.OFFSEASON_BOT)
            instance = new Intake();
        else
            instance = new EmptyIntake();
    }
    
    public static AbstractIntake getInstance() {
        return instance;
    }

    public abstract void setSpeed(double topSpeed, double bottomSpeed);

    public void childPeriodic() {};

    @Override
    public void periodic() {
        childPeriodic();
    }
}