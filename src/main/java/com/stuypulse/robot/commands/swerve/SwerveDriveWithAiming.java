package com.stuypulse.robot.commands.swerve;

import static com.stuypulse.robot.constants.Settings.Alignment.TARGET_DISTANCE_IN;

import com.stuypulse.robot.constants.Field;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveWithAiming extends Command {

    private final SwerveDrive swerve;

    private VStream speed;
    private final AngleController angleController;
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

        this.angleController = new AnglePIDController(Alignment.Rotation.P, Alignment.Rotation.I, Alignment.Rotation.D);
        this.target = target;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        Pose2d robotPose = AbstractOdometry.getInstance().getPose();

        Rotation2d target = new Rotation2d(
                                robotPose.getX() - this.target.getX(), 
                                robotPose.getY() - this.target.getY());
            
        double angularVel = -angleController.update(
                            Angle.fromRotation2d(target),
                            Angle.fromRotation2d(swerve.getGyroAngle()));

        Pose2d speaker = Field.getSpeakerPose();
        Pose2d robot = AbstractOdometry.getInstance().getPose();
        Vector2D speakerPos = new Vector2D(speaker.getX(), speaker.getY());
        Vector2D robotPos = new Vector2D(robot.getX(), robot.getY());
        Vector2D targetPos = speakerPos.add(robotPos.sub(speakerPos).normalize().mul(Units.inchesToMeters(TARGET_DISTANCE_IN.get())));
        Rotation2d targetAngle = targetPos.getTranslation2d().minus(robotPos.getTranslation2d()).getAngle().plus(Rotation2d.fromDegrees(180));
        SmartDashboard.putNumber("Vision/To Score/Target Angle", targetAngle.getDegrees());

        swerve.drive(speed.get(), angularVel);
    }
}
