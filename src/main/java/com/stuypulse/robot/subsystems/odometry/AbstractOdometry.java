package com.stuypulse.robot.subsystems.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractOdometry extends SubsystemBase {
    private static final AbstractOdometry instance;

    static { 
        instance = new Odometry();
    }
    
    public static AbstractOdometry getInstance() {
        return instance;
    }

    public abstract void reset(Pose2d pose);
    public abstract Field2d getField();
    public abstract Pose2d getPose();

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public Translation2d getTranslation() {
        return getPose().getTranslation();
    }
}