package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveWhileShooting extends Command {

    private final SwerveDrive swerve;
    private final AbstractOdometry odometry;

    private VStream speed;
    private final AngleController controller;
    private final Pose2d target;

    private VStream robotSpeed;

    public SwerveDriveWhileShooting(Pose2d target, Gamepad driver) {
        this.swerve = SwerveDrive.getInstance();
        this.odometry = AbstractOdometry.getInstance();

        this.speed = VStream.create(driver::getLeftStick)
                .filtered(
                    new VDeadZone(Drive.DEADBAND),
                    x -> x.clamp(1.0),
                    x -> Settings.vpow(x, Drive.POWER.get()),
                    x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                    new VRateLimit(Drive.MAX_TELEOP_ACCEL),
                    new VLowPassFilter(Drive.RC)
                );
        
        this.controller = new AnglePIDController(Alignment.Rotation.P, Alignment.Rotation.I, Alignment.Rotation.D);
        this.target = target;

        this.robotSpeed = VStream.create(() -> chassisSpeedsVector())
            .filtered(new VLowPassFilter(Settings.Swerve.VELOCITY_RC.get()))
            .filtered(new VRateLimit(3));
        
        odometry.getField().getObject("Target").setPose(target);

        addRequirements(swerve);
    }

    private Vector2D chassisSpeedsVector() {
        ChassisSpeeds speeds = swerve.getChassisSpeeds();
        return new Vector2D(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond
        );
    }
   
    public Vector2D getVelocity(){
        return robotSpeed.get();
    }

    public Vector2D getFieldRelativeSpeed(){
        return robotSpeed.get().rotate(Angle.fromRotation2d(odometry.getPose().getRotation()));
    }

    @Override
    public void execute() {

        //suspicion: getVelocity is wrong
        Vector2D velNoteInitial = new Vector2D(new Translation2d(Settings.Swerve.NOTE_VELOCITY, odometry.getPose().getRotation())); 
        



        Pose2d robotPose = odometry.getPose();
        Vector2D targetVector = new Vector2D(
                            this.target.getX() - robotPose.getX(), 
                            this.target.getY() - robotPose.getY());

        Vector2D velocityDifference = targetVector.sub(getFieldRelativeSpeed()); 
    
        double angularVel = -controller.update(
                            velocityDifference.getAngle(),
                            Angle.fromRotation2d(odometry.getRotation()));
        

        swerve.drive(speed.get(), angularVel);

        SmartDashboard.putNumber("Swerve/unadjusted angle", targetVector.getAngle().toDegrees());
        SmartDashboard.putNumber("Swerve/targetVector x", targetVector.x);
        SmartDashboard.putNumber("Swerve/targetVector y", targetVector.y);

        SmartDashboard.putNumber("Swerve/adjusted (subtracted) angle", velocityDifference.getAngle().toDegrees());
        SmartDashboard.putNumber("Swerve/angle setpoint (from odometry)", odometry.getRotation().getDegrees());
        SmartDashboard.putNumber("Swerve/getFieldRelativeSpeed x", getFieldRelativeSpeed().x);
        SmartDashboard.putNumber("Swerve/getFieldRelativeSpeed y", getFieldRelativeSpeed().y);

        //optimal velocity = target vector.sub(getFieldRelativeSpeedPoints)

        

    }
}
