package com.stuypulse.robot.commands.swerve;

import static com.stuypulse.robot.constants.Settings.Alignment.TARGET_DISTANCE_IN;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Assist;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveWithAiming extends Command {

    private final SwerveDrive swerve;

    private VStream speed;
    private final AngleController angleController;
    private final Translation2d target;

    public SwerveDriveWithAiming(Translation2d target, Gamepad driver) {
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
        double xPosition = swerve.getChassisSpeeds().vxMetersPerSecond * Assist.TIME;
        double yPosition = swerve.getChassisSpeeds().vyMetersPerSecond * Assist.TIME;
        Translation2d position = new Translation2d(xPosition, yPosition);
        Translation2d robotPose = AbstractOdometry.getInstance().getPose().getTranslation().plus(position);

        Rotation2d target = robotPose.minus(this.target).getAngle();
            
        double angularVel = -angleController.update(
            Angle.fromRotation2d(target),
            Angle.fromRotation2d(AbstractOdometry.getInstance().getRotation()));

        swerve.drive(speed.get(), angularVel);
    }
}
