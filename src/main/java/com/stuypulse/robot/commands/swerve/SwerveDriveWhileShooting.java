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
            .filtered(new VRateLimit(5));
        
        odometry.getField().getObject("Target").setPose(target);

        addRequirements(swerve);
    }

    private Vector2D chassisSpeedsVector() {
        return  new Vector2D(
            swerve.getChassisSpeeds().vxMetersPerSecond, 
            swerve.getChassisSpeeds().vyMetersPerSecond
        );
        
    }
   
    public Vector2D getVelocity(){
        return robotSpeed.get();
    }


    @Override
    public void execute() {
         Vector2D velNote = new Vector2D(new Translation2d(Settings.Swerve.NOTE_VELOCITY, odometry.getRotation())) // field relative 
            .sub(getVelocity()); 
        //it should point in the opposite direction of velocity

        Rotation2d correctionAngle = velNote.getAngle().getRotation2d().minus(odometry.getRotation());
        Translation2d h = new Translation2d(Settings.Swerve.NOTE_VELOCITY, odometry.getRotation());

        Pose2d robotPose = AbstractOdometry.getInstance().getPose();
        

        Rotation2d target = new Rotation2d(
                                this.target.getX() - robotPose.getX(), 
                                this.target.getY() - robotPose.getY())
                                .plus(correctionAngle.times(
                                    //Math.signum(swerve.getChassisSpeeds().omegaRadiansPerSecond) //clockwise or ccw
                                    Math.signum(- odometry.getRotation().getDegrees()) 
                                    * Math.signum(-swerve.getChassisSpeeds().vxMetersPerSecond) *
                                    Math.signum(-swerve.getChassisSpeeds().vyMetersPerSecond) // we want to move in the opposite direction
                                    )
                                    );

                                //.minus(getVelocity().getAngle().getRotation2d()); // pointing at where we want it to
            
        double angularVel = -controller.update(
                            Angle.fromRotation2d(target),
                            Angle.fromRotation2d(odometry.getRotation()));

        swerve.drive(speed.get(), angularVel);

        SmartDashboard.putNumber("preaim angle", velNote.getAngle().toDegrees());
        SmartDashboard.putNumber("Swerve/notevx", h.getX());
        SmartDashboard.putNumber("Swerve/notevy", h.getY());
        SmartDashboard.putNumber("Swerve/target Angle (calculated)", target.getDegrees());
        SmartDashboard.putNumber("Swerve/angle setpoint (from odometry)", odometry.getRotation().getDegrees());
        SmartDashboard.putNumber("Swerve/angle added", correctionAngle.getDegrees());
        SmartDashboard.putNumber("Swerve/velocity angle", velNote.getAngle().toDegrees());

    }
}
