package com.stuypulse.robot.util;

import static com.stuypulse.robot.constants.Cameras.Limelight.*;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.NoteDetection;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This class handles interactions between the robot code 
 * and the Limelight system through the NetworkTables.
 */
public class Limelight {
    
    private final DoubleEntry txEntry;
    private final DoubleEntry tyEntry;
    private final IntegerEntry tvEntry;

    private int limelightID;
    private Pose3d robotRelativePose;

    private double txData;
    private double tyData;

    private IStream xAngle;
    private BStream noteData;

    public Limelight(String tableName, Pose3d robotRelativePose) {

        this.robotRelativePose = robotRelativePose;

        for(int i = 0; i < LIMELIGHTS.length; i++) {
            if(LIMELIGHTS[i].equals(tableName)) {
                limelightID = i;
            }
        }

        NetworkTable limelight = NetworkTableInstance.getDefault().getTable(tableName);

        txEntry = limelight.getDoubleTopic("tx").getEntry(0.0);
        tyEntry = limelight.getDoubleTopic("ty").getEntry(0.0);
        tvEntry = limelight.getIntegerTopic("tv").getEntry(0);

        xAngle = IStream.create(() -> txData)
            .filtered(new LowPassFilter(NoteDetection.X_ANGLE_RC));
        noteData = BStream.create(() -> tvEntry.get() == 1)
            .filtered(new BDebounceRC.Both(NoteDetection.DEBOUNCE_TIME));
    }

    /**
     * Returnns whether or not there is data from the Limelight.
     * @return whether or not there is data from the Limelight
     */
    public boolean hasNoteData() {
        return noteData.get();
    }

    /**
     * Updates the data from the Limelight.
     */
    public void updateData() {
        if (tvEntry.get() == 1) {
            txData = txEntry.get();
            tyData = tyEntry.get();
        }
    }

    /**
     * Returns the position of the note relative to the robot.
     * @return the position of the note relative to the robot
     */
    public Pose3d getRobotRelativePose() {
        return robotRelativePose;
    }

    /**
     * Returns the x angle of the note relative to the robot.
     * @return the x angle of the note relative to the robot
     */
    public double getXAngle() {
        return xAngle.get() - Units.radiansToDegrees(POSITIONS[limelightID].getRotation().getZ());
    }

    /**
     * Returns the y angle of the note relative to the robot.
     * @return the y angle of the note relative to the robot
     */
    public double getYAngle() {
        return tyData - Units.radiansToDegrees(POSITIONS[limelightID].getRotation().getY());
    }

    // limelight targets far end of note, so have to subtract half of note length
    public double getDistanceToNote() {
        Rotation2d yRotation = Rotation2d.fromDegrees(getYAngle());
        return POSITIONS[limelightID].getZ() / -yRotation.getTan() + POSITIONS[limelightID].getX() - Field.NOTE_LENGTH / 2.0;
    }
}