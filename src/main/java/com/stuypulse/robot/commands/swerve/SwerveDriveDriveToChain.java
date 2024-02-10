package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDriveToChain extends Command {

    private final AbstractOdometry odometry;
    private final SwerveDrive swerve;

    private Pose2d trapPose;
    
    public SwerveDriveDriveToChain() {
        odometry = AbstractOdometry.getInstance();
        swerve = SwerveDrive.getInstance();

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        trapPose = Field.getClosestAllianceTrapPose(odometry.getPose());

        odometry.getField().getObject("Trap").setPose(trapPose);
    }

    @Override
    public void execute() {
        Rotation2d translationAngle = trapPose.getTranslation().minus(odometry.getPose().getTranslation()).getAngle();

        Translation2d translation = new Translation2d(Alignment.INTO_CHAIN_SPEED.get(), translationAngle);

        swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), translation.getY(), 0, odometry.getRotation()));
    }

    private double getDistanceToTrap() {
        return odometry.getPose().minus(trapPose).getTranslation().getNorm();
    }

    @Override
    public boolean isFinished() {
        return getDistanceToTrap() <= Alignment.TRAP_CLIMB_DISTANCE.get();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        odometry.getField().getObject("Trap").setPose(new Pose2d(Double.NaN, Double.NaN, new Rotation2d(Double.NaN)));
    }

}
