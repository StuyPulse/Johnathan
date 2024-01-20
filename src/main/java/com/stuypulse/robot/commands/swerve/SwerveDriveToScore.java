package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;

import static com.stuypulse.robot.constants.Settings.Alignment.*;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveToScore extends Command {
    
    private final SwerveDrive swerve;

    private final HolonomicController controller;
    private Pose2d target;
    private final FieldObject2d targetPose2d;

    public SwerveDriveToScore() {
        this.swerve = SwerveDrive.getInstance();

        controller = new HolonomicController(
            new PIDController(Translation.P, Translation.I, Translation.D),
            new PIDController(Translation.P, Translation.I, Translation.D),
            new AnglePIDController(Rotation.P, Rotation.I, Rotation.D));

        SmartDashboard.putData("Alignment/Score Controller", controller);
        targetPose2d = AbstractOdometry.getInstance().getField().getObject("Target Pose");

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        Pose2d speaker = Field.getSpeakerPose();
        Pose2d robot = AbstractOdometry.getInstance().getPose();
        Vector2D speakerPos = new Vector2D(speaker.getX(), speaker.getY());
        Vector2D robotPos = new Vector2D(robot.getX(), robot.getY());
        Vector2D targetPos = speakerPos.add(robotPos.sub(speakerPos).normalize().mul(Units.inchesToMeters(TARGET_DISTANCE_IN.get())));
        this.target = new Pose2d(targetPos.x, targetPos.y, new Rotation2d(Math.atan2(targetPos.y - robot.getY(), targetPos.x - robot.getX())));
    }

    @Override
    public void execute() {
        controller.update(target, AbstractOdometry.getInstance().getPose());
        swerve.setChassisSpeeds(controller.getOutput());
        targetPose2d.setPose(target);
    }

    @Override
    public boolean isFinished() {
        return controller.isDone(X_TOLERANCE.get(), Y_TOLERANCE.get(), ANGLE_TOLERANCE.get());
    }

    @Override
    public void end(boolean interrupted) {
        this.swerve.stop();
    }
}
