package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.NoteDetection;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Assist;
import com.stuypulse.robot.subsystems.notevision.AbstractNoteVision;
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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveNoteAlignedDrive extends Command {
       
    private final SwerveDrive swerve;
    private final AbstractOdometry odometry;
    private final AbstractNoteVision noteVision;
    
    private final VStream speed; 

    private final AngleController alignController;

    public SwerveDriveNoteAlignedDrive(Gamepad driver) {
        swerve = SwerveDrive.getInstance();
        odometry = AbstractOdometry.getInstance();
        noteVision = AbstractNoteVision.getInstance();

        speed = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1.0),
                x -> Settings.vpow(x, Drive.POWER.get()),
                x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL),
                new VLowPassFilter(Drive.RC)
            );  

        alignController = new AnglePIDController(NoteDetection.Rotation.P, NoteDetection.Rotation.I, NoteDetection.Rotation.D);
        
        addRequirements(swerve);
    }

   @Override
    public void execute() {
        double xPosition = swerve.getChassisSpeeds().vxMetersPerSecond * Assist.TIME;
        double yPosition = swerve.getChassisSpeeds().vyMetersPerSecond * Assist.TIME;
        Translation2d position = new Translation2d(xPosition, yPosition);

        Translation2d targetTranslation = odometry.getTranslation()
            .plus(position)
            .plus(new Translation2d(Swerve.CENTER_TO_INTAKE_FRONT, 0).rotateBy(odometry.getRotation()));

        Rotation2d targetRotation = noteVision.getEstimatedNoteTranslation().minus(targetTranslation).getAngle();

        double angularVel = -alignController.update(
            Angle.fromRotation2d(targetRotation),
            Angle.fromRotation2d(odometry.getRotation())
        );
        
        // robot relative
        swerve.drive(speed.get(), angularVel);

        SmartDashboard.putNumber("Note Detection/Angle Output", alignController.getOutput());

    }
}