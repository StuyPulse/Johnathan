package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.NoteDetection;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Driver.Turn;
import com.stuypulse.robot.subsystems.notevision.AbstractNoteVision;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveNoteAlignedDrive extends Command {
       
    private SwerveDrive swerve;
    private AbstractNoteVision noteVision;
    
    private VStream speed; 
    private IStream turn;

    private AngleController alignController;

    public SwerveDriveNoteAlignedDrive(Gamepad driver) {
        swerve = SwerveDrive.getInstance();
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

        turn = IStream.create(driver::getRightX)
            .filtered(
                x -> SLMath.deadband(x, Turn.DEADBAND.get()),
                x -> SLMath.spow(x, Turn.POWER.get()),
                x -> x * Turn.MAX_TELEOP_TURNING.get(),
                new LowPassFilter(Turn.RC)
            );

        alignController = new AnglePIDController(NoteDetection.Rotation.P, NoteDetection.Rotation.I, NoteDetection.Rotation.D);
        
        addRequirements(swerve);
    }

   @Override
    public void execute() {
        double angularVel = turn.get();

        if (noteVision.hasNoteData() && Math.abs(noteVision.getRotationToNote().getDegrees()) > NoteDetection.THRESHOLD_ANGLE.get()) {
            angularVel = -alignController.update(
                Angle.kZero,
                Angle.fromRotation2d(noteVision.getRotationToNote())
            );
        }
        
        // robot relative
        swerve.setChassisSpeeds(new ChassisSpeeds(speed.get().x, speed.get().y, angularVel));

        SmartDashboard.putNumber("Note Vision/Output", alignController.getOutput());

    }
}