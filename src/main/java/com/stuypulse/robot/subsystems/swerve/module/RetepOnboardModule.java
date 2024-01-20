package com.stuypulse.robot.subsystems.swerve.module;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Encoder;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;
import com.stuypulse.stuylib.streams.numbers.filters.Derivative;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RetepOnboardModule extends AbstractModule {

    // data
    private final String id;
    private final Translation2d translationOffset;
    private final Rotation2d angleOffset;
    private SwerveModuleState targetState;

    // turn
    private final CANSparkMax turnMotor; 
    private final CANcoder turnEncoder;

    // drive
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;

    // controllers
    private final SparkPIDController turnPID;
    private final SparkPIDController drivePID;

    // drive acceleration
    private final Derivative driveAccel;
    
    public RetepOnboardModule(String id, Translation2d translationOffset, Rotation2d angleOffset, int turnID, int driveID, int encoderID) {
        this.id = id;
        this.translationOffset = translationOffset; 
        this.angleOffset = angleOffset;
        
        /*** TURN ***/
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        turnEncoder = new CANcoder(encoderID);
        turnMotor.setIdleMode(IdleMode.kBrake);

        turnMotor.getEncoder().setPosition(MathUtil.inputModulus(getAngle().getRotations(), 0, 1));

        turnPID = turnMotor.getPIDController();
        turnPID.setFeedbackDevice(turnMotor.getEncoder());

        turnPID.setP(Turn.kP.get());
        turnPID.setI(Turn.kI.get());
        turnPID.setD(Turn.kD.get());
        turnPID.setFF(0);
        turnPID.setIZone(0);
        turnPID.setOutputRange(-1, 1);

        turnPID.setPositionPIDWrappingEnabled(true);
        turnPID.setPositionPIDWrappingMinInput(Rotation2d.fromDegrees(0).getRotations());
        turnPID.setPositionPIDWrappingMaxInput(Rotation2d.fromDegrees(360).getRotations());

        turnMotor.enableVoltageCompensation(12);

        Motors.Swerve.TURN_CONFIG.configure(turnMotor);

        /*** DRIVE ***/
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveMotor.setIdleMode(IdleMode.kBrake);

        driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);

        drivePID = driveMotor.getPIDController();
        drivePID.setFeedbackDevice(driveEncoder);

        drivePID.setP(Drive.kP.get());
        drivePID.setI(Drive.kI.get());
        drivePID.setD(Drive.kD.get());
        drivePID.setFF(Drive.kV.get());
        drivePID.setIZone(0);

        driveMotor.enableVoltageCompensation(12);
        
        Motors.Swerve.DRIVE_CONFIG.configure(driveMotor);

        driveAccel = new Derivative();

        targetState = new SwerveModuleState();
    }

    public Translation2d getOffset() {
        return translationOffset;
    }

    public String getID() {
        return id;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnEncoder.getAbsolutePosition().getValueAsDouble()).minus(angleOffset);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getAngle());
    }

    @Override
    public void periodic() {
        if((Turn.kP.get() != turnPID.getP())) turnPID.setP(Turn.kP.get());
        if((Turn.kI.get() != turnPID.getP())) turnPID.setI(Turn.kI.get());
        if((Turn.kD.get() != turnPID.getP())) turnPID.setD(Turn.kD.get());

        if((Drive.kP.get() != drivePID.getP())) drivePID.setP(Drive.kP.get());
        if((Drive.kI.get() != drivePID.getP())) drivePID.setI(Drive.kI.get());
        if((Drive.kD.get() != drivePID.getP())) drivePID.setD(Drive.kD.get());
        if((Drive.kV.get() != drivePID.getP())) drivePID.setFF(Drive.kV.get());

        double kSVoltage = Drive.kS.get() * Math.signum(targetState.speedMetersPerSecond);
        double kAVoltage = Drive.kA.get() * driveAccel.get(targetState.speedMetersPerSecond);

        turnPID.setReference(targetState.angle.getRotations(), ControlType.kPosition);
        drivePID.setReference(targetState.speedMetersPerSecond, ControlType.kVelocity, 0, kSVoltage + kAVoltage);

        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Drive Speed", driveMotor.get());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Turn Speed", turnMotor.get());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Angle Error", targetState.angle.minus(getAngle()).getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Target Speed", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Speed", getVelocity());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Raw Encoder Angle", Units.rotationsToDegrees(turnEncoder.getAbsolutePosition().getValueAsDouble()));
    }
}

