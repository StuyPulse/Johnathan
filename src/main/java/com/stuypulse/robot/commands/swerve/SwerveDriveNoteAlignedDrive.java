package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Driver.Turn;
import com.stuypulse.robot.constants.Settings .Driver.Turn.GyroFeedback;
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

import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveNoteAlignedDrive extends Command {
       
    private SwerveDrive swerve;
    private AbstractNoteVision noteVision;
    
    private VStream speed; 
    private IStream turn;

    private final Gamepad driver;

    private AngleController alignController;

    public SwerveDriveNoteAlignedDrive(Gamepad driver) {
        this.driver = driver;

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

        alignController = new AnglePIDController(GyroFeedback.P, GyroFeedback.I, GyroFeedback.D);
        
        addRequirements(swerve);
    }

   @Override
    public void execute() {
        double angularVel = turn.get();

        if (noteVision.hasNoteData()) {
            angularVel = -alignController.update(
                Angle.kZero,
                Angle.fromRotation2d(noteVision.getRotationToNote())
            );
        }

        if(driver.getRawStartButton() || driver.getRawSelectButton()) {
            swerve.setXMode();
        }
        else {
            swerve.drive(speed.get(), angularVel);
        }

    }
}