package com.stuypulse.robot.commands.leds;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stuypulse.robot.constants.LEDColor;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.LED;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.leds.instructions.LEDInstruction;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.vision.AbstractVision;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDAlign extends Command implements LEDInstruction {
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    private int index;
    private final LEDController ledController;
    private final AbstractOdometry odometry;
    private final Pose2d startPose;
    
    private final BStream isXAligned;
    private final BStream isYAligned;
    private final BStream isThetaAligned;

    public LEDAlign(PathPlannerAuto auton) {
        startPose = PathPlannerAuto.getStaringPoseFromAutoFile(auton.getName()); /*Field.getSpeakerPose()*/;
        odometry = AbstractOdometry.getInstance();
        ledController = LEDController.getInstance();

        isXAligned = BStream.create(this::isXAligned)     
            .filtered(new BDebounceRC.Both(Alignment.DEBOUNCE_TIME.get()));
        isYAligned = BStream.create(this::isYAligned)     
            .filtered(new BDebounceRC.Both(Alignment.DEBOUNCE_TIME.get()));
        isThetaAligned = BStream.create(this::isThetaAligned)     
            .filtered(new BDebounceRC.Both(Alignment.DEBOUNCE_TIME.get()));

        addRequirements(ledController);
    }
   
    public boolean isXAligned() {
        return Math.abs(odometry.getPose().getX() - startPose.getX()) < Settings.Alignment.X_TOLERANCE.get();
    }

    public boolean isYAligned() {
        return Math.abs(odometry.getPose().getY() - startPose.getY()) < Settings.Alignment.Y_TOLERANCE.get();
    }
    
    public boolean isThetaAligned() {
        return Math.abs(odometry.getPose().getRotation().getDegrees() - startPose.getRotation().getDegrees()) < Settings.Alignment.ANGLE_TOLERANCE.get();
    }

  
    @Override
    public void execute() {
       ledController.forceSetLED(this);
    }
   
    /* 
    @Override
    public boolean isFinished() {
        return isXAligned.get() && isYAligned.get() && isThetaAligned.get();
    }
  
    @Override
    public void end(boolean interrupted) {
        ledController.forceSetLED(LEDColor.RAINBOW);
    }
    */
 
    @Override
    public void setLED(AddressableLEDBuffer ledsBuffer) {
        Pose2d pose = odometry.getPose();
        int middleLEDindex = ledsBuffer.getLength() / 2;
        ledsBuffer.setLED(middleLEDindex, Color.kBlack);

        if (isXAligned() && isYAligned() && isThetaAligned()) {
            ledController.forceSetLED(LEDColor.RAINBOW);
        } else {
            int index = middleLEDindex;

            if (!isXAligned.get()) {
                ledController.forceSetLED(LEDColor.RED);
                index = linearInterp(pose.getX(), startPose.getX(), LED.TRANSLATION_SPREAD.get());
            }
            if (!isYAligned.get() && isXAligned()) {
                ledController.forceSetLED(LEDColor.GREEN);
                index = linearInterp(pose.getY(), startPose.getY(), LED.TRANSLATION_SPREAD.get());
            }
            if (!isThetaAligned.get() && isXAligned() && isYAligned()) {
                ledController.forceSetLED(LEDColor.DARK_BLUE);
                index = linearInterp(
                    pose.getRotation().getDegrees(),
                    startPose.getRotation().getDegrees(),
                    LED.ROTATION_SPREAD.get());
            }
            ledsBuffer.setLED(index, Color.kWhite);
        }
        if (AbstractVision.getInstance().getOutput().isEmpty()) LEDColor.WHITE.setLED(ledsBuffer);
    }

    private int linearInterp(double robotMeasurement, double targetPos, double spread) {
        double lowerBound = targetPos - spread;
        double upperBound = targetPos + spread;
        if (robotMeasurement < lowerBound) {
            return 0;
        }
        if (robotMeasurement > upperBound) {
            return Settings.LED.LED_LENGTH - 1;
        }
        return (int) (Settings.LED.LED_LENGTH * (robotMeasurement - lowerBound) / (upperBound - lowerBound));
    }
}
