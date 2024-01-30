package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
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
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveWhileShooting extends Command {

    private final SwerveDrive swerve;

    private VStream speed;
    private final AngleController gyroFeedback;
    private final Pose2d target;

   
    private VStream robotSpeed;

    public SwerveDriveWhileShooting(Pose2d target, Gamepad driver) {
        this.swerve = SwerveDrive.getInstance();

        this.speed = VStream.create(driver::getLeftStick)
                .filtered(
                    new VDeadZone(Drive.DEADBAND),
                    x -> x.clamp(1.0),
                    x -> Settings.vpow(x, Drive.POWER.get()),
                    x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                    new VRateLimit(Drive.MAX_TELEOP_ACCEL),
                    new VLowPassFilter(Drive.RC)
                );
        
        this.gyroFeedback = new AnglePIDController(Alignment.Gyro.P.get(), Alignment.Gyro.I.get(), Alignment.Gyro.D.get());
        this.target = target;

        this.robotSpeed = VStream.create(() -> chassisSpeedsVector())
            .filtered(new VLowPassFilter(Settings.Swerve.VELOCITY_RC.get()));

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
        
        
        Vector2D velNote = new Vector2D(new Translation2d(Settings.Swerve.NOTE_VELOCITY, swerve.getGyroAngle())) // field relative 
            .sub(getVelocity()) // get the true note velocity 
            .sub(getVelocity()); // shoot where we inteded to

       

        Pose2d robotPose = AbstractOdometry.getInstance().getPose();

        Rotation2d target = new Rotation2d(
                                robotPose.getX() - this.target.getX(), 
                                robotPose.getY() - this.target.getY())
                                .plus(velNote.getAngle().getRotation2d()); // pointing at where we want it to
            
        double angularVel = -gyroFeedback.update(
                            Angle.fromRotation2d(target),
                            Angle.fromRotation2d(swerve.getGyroAngle()));

        swerve.drive(speed.get(), angularVel);

       
    }
}
