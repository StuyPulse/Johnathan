/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.swerve.module;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.angles.filters.ARateLimit;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.MatchState;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Motors.CANSparkMaxConfig;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;


public class JimSwerveModule extends SwerveModule {

    private static final double WIDTH = Units.inchesToMeters(26.504);
    private static final double LENGTH = Units.inchesToMeters(20.508);

    private static final double MAX_TURNING = 6.28;

    private interface Encoder {
        public interface Drive {
            double WHEEL_DIAMETER = Units.inchesToMeters(3);
            double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
            double GEAR_RATIO = 1.0 / 4.71;

            double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
            double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
        }

        public interface Turn {
            double POSITION_CONVERSION = 1;
            double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
        }
    }

    private interface FrontRight {
        String ID = "Front Right";
        Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(122.949092) // recalibrated 11/11        
            .plus(Rotation2d.fromDegrees(0));
        Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * -0.5);
    }

    private interface FrontLeft {
        String ID = "Front Left";
        Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(249.731491) // recalibrated 3/24
            .plus(Rotation2d.fromDegrees(270));
        Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * +0.5);
    }

    private interface BackLeft {
        String ID = "Back Left";
        Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(125.371964) // recalibrated 4/6
            .plus(Rotation2d.fromDegrees(180));
        Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * +0.5);
    }

    private interface BackRight {
        String ID = "Back Right";
        Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(117.311604) // recalibrated 10/1
            .plus(Rotation2d.fromDegrees(90));
        Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * -0.5);
    }

    public interface Turn {
        SmartNumber kP = new SmartNumber("Swerve/Turn/kP", 3.5);
        double kI = 0.0;
        SmartNumber kD = new SmartNumber("Swerve/Turn/kD", 0.1);

        SmartNumber kV = new SmartNumber("Swerve/Turn/kV", 0.25);
        SmartNumber kA = new SmartNumber("Swerve/Turn/kA", 0.007);
    }

    public interface Drive {
        double kP = 0.8;
        double kI = 0.0;
        double kD = 0.0;

        double kS = 0.22304;
        double kV = 2.4899;
        double kA = 0.41763;
    }

    /** CODE **/

    public static SwerveDrive getModules() {
        return new SwerveDrive(
            new JimSwerveModule(FrontRight.ID, FrontRight.MODULE_OFFSET, 11, FrontRight.ABSOLUTE_OFFSET, 10),
            new JimSwerveModule(FrontLeft.ID, FrontLeft.MODULE_OFFSET, 13, FrontLeft.ABSOLUTE_OFFSET, 12),
            new JimSwerveModule(BackLeft.ID, BackLeft.MODULE_OFFSET, 15, BackLeft.ABSOLUTE_OFFSET, 14),
            new JimSwerveModule(BackRight.ID, BackRight.MODULE_OFFSET, 17, BackRight.ABSOLUTE_OFFSET, 16)
        );
    }

    // module data
    private final String id;
    private final Translation2d location;
    private SwerveModuleState targetState;

    // turn
    private final CANSparkMax turnMotor;
    private final SparkAbsoluteEncoder absoluteEncoder;

    // drive
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;

    private Rotation2d angleOffset;

    // controllers
    private Controller driveController;
    private AngleController turnController;

    public JimSwerveModule(String id, Translation2d location, int turnCANId, Rotation2d angleOffset, int driveCANId) {

        // module data
        this.id = id;
        this.location = location;

        // turn
        turnMotor = new CANSparkMax(turnCANId, MotorType.kBrushless);

        // double check this
        absoluteEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(Encoder.Turn.POSITION_CONVERSION);
        absoluteEncoder.setVelocityConversionFactor(Encoder.Turn.VELOCITY_CONVERSION);
        absoluteEncoder.setZeroOffset(0.0);

        // TODO: add as configurable setting
        absoluteEncoder.setInverted(true);
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        Motors.disableStatusFrames(turnMotor, 3, 4);

        turnController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
            .setSetpointFilter(new ARateLimit(MAX_TURNING));

        this.angleOffset = angleOffset;

        // drive
        driveMotor = new CANSparkMax(driveCANId, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);
        Motors.disableStatusFrames(driveMotor, 3, 4, 5, 6);

        driveController = new PIDController(Drive.kP, Drive.kI, Drive.kD)
                .setOutputFilter(x -> Robot.getMatchState() == MatchState.TELEOP ? 0 : x)
            .add(new MotorFeedforward(Drive.kS, Drive.kV, Drive.kA).velocity());

        targetState = new SwerveModuleState();

        new CANSparkMaxConfig(false, IdleMode.kBrake, 60, 0).configure(turnMotor);
        new CANSparkMaxConfig(false, IdleMode.kBrake, 20, 0).configure(turnMotor);
    }

    @Override
    public String getID() {
        return id;
    }

    @Override
    public Translation2d getOffset() {
        return location;
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    private double getVelocity() {
        return driveEncoder.getVelocity();
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRotations(absoluteEncoder.getPosition()).minus(angleOffset);
    }

    @Override
    public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getAngle());
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    @Override
    public void periodic() {
        // turn
        turnMotor.setVoltage(turnController.update(
            Angle.fromRotation2d(targetState.angle),
            Angle.fromRotation2d(getAngle())));

        // drive
        driveMotor.setVoltage(driveController.update(
            targetState.speedMetersPerSecond,
            getVelocity()));

        SmartDashboard.putNumber("Swerve/" + id + "/Raw Angle (deg)", Units.rotationsToDegrees(absoluteEncoder.getPosition()));
        SmartDashboard.putNumber("Swerve/" + id + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle Error", turnController.getError().toDegrees());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle Voltage", turnController.getOutput());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle Current", turnMotor.getOutputCurrent());
        SmartDashboard.putNumber("Swerve/" + id + "/Target Velocity", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity", getVelocity());
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity Error", driveController.getError());
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity Voltage", driveController.getOutput());
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity Current", driveMotor.getOutputCurrent());
    }
}
