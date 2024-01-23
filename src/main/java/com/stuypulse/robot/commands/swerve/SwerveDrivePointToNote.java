package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings.NoteDetection;
import com.stuypulse.robot.subsystems.notevision.AbstractNoteVision;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDrivePointToNote extends Command {

    private final SwerveDrive swerve;
    private final AbstractNoteVision noteVision;

    private final AngleController alignController;

    public SwerveDrivePointToNote() {
        swerve = SwerveDrive.getInstance();
        noteVision = AbstractNoteVision.getInstance();

        alignController = new AnglePIDController(NoteDetection.Rotation.P, NoteDetection.Rotation.I, NoteDetection.Rotation.D);
        
        addRequirements(swerve);
    }

   @Override
    public void execute() {
        double angularVel = -alignController.update(
            Angle.kZero,
            Angle.fromRotation2d(noteVision.getRotationToNote())
        );
        
        swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, -angularVel));

        SmartDashboard.putNumber("Note Detection/Angular Vel Output", alignController.getOutput());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(noteVision.getRotationToNote().getDegrees()) < NoteDetection.THRESHOLD_ANGLE.get();
    }
}