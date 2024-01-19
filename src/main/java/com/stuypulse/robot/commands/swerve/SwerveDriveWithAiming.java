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
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveWithAiming extends Command {

    private final SwerveDrive swerve;

    private VStream speed;
    private final AngleController gyroFeedback;
    private final Pose2d target;

    public SwerveDriveWithAiming(Pose2d target, Gamepad driver) {
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

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        
        Pose2d robotPose = AbstractOdometry.getInstance().getPose();

        Rotation2d target = new Rotation2d(
                                robotPose.getX() - this.target.getX(), 
                                robotPose.getY() - this.target.getY())
                                .minus(Rotation2d.fromDegrees(180));
            
        double angularVel = -gyroFeedback.update(
                            Angle.fromRotation2d(target.plus(Rotation2d.fromDegrees(180))),
                            Angle.fromRotation2d(swerve.getGyroAngle()));

        swerve.drive(speed.get(), angularVel);
    }
}
