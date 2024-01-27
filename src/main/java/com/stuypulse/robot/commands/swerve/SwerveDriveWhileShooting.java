package com.stuypulse.robot.commands.swerve;

import java.util.ArrayList;
import java.util.LinkedList;

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
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveWhileShooting extends Command {

    private final SwerveDrive swerve;

    private VStream speed;
    private final AngleController gyroFeedback;
    private final Pose2d target;

    private LinkedList<Vector2D> velocities = new LinkedList<Vector2D>();
    private SmartNumber magVelNote = new SmartNumber("Note Velocity", 8);//should be in settings

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

        addRequirements(swerve);
    }


   
    private Vector2D avgVel(){
        //updating the linked list
        velocities.add(speed.get());
        if(velocities.size() > 10){
            velocities.removeFirst();
        }

        Vector2D avgVel = new Vector2D(0, 0);
        //calculate average velocity
        for(Vector2D vel : velocities){
            avgVel.add(vel);
        }
        return avgVel.div(velocities.size());
    }


    @Override
    public void execute() {
        
        
        Vector2D velNote = new Vector2D(
            magVelNote.get() * Math.cos(swerve.getGyroAngle().getRadians()), 
            magVelNote.get() * Math.sin(swerve.getGyroAngle().getRadians())
        );

        //subracting to get the true vector
        Vector2D resultant = velNote.sub(avgVel());
        Rotation2d angle = Rotation2d.fromRadians(Math.atan2(resultant.y, resultant.x));
       

        Pose2d robotPose = AbstractOdometry.getInstance().getPose();

        Rotation2d target = new Rotation2d(
                                robotPose.getX() - this.target.getX(), 
                                robotPose.getY() - this.target.getY())
                                .plus(angle);
            
        double angularVel = -gyroFeedback.update(
                            Angle.fromRotation2d(target),
                            Angle.fromRotation2d(swerve.getGyroAngle()));

        swerve.drive(speed.get(), angularVel);

       
    }
}
/*
 * get the average velocity for the past 10 ticks (0.2 seconds) as a vector
 * get the velocity of the speaker as a vector (you want x and y), which means 
 * you need the angle from the target to the current pose (multiply the velocity by sintheta and cos to get x and y)
 * subtract the robot vector from the note vector 
 * get the resulting vector's field relative angle
 * add that angle to the target angle 
 */