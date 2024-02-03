/************************ PROJECT JON *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import java.util.Optional;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class OV2311Camera {

    private final Field2d field;

    private final String name;
    private final Pose3d offset;

    // Default Values
    private final int camera_id = 0;
    private final int camera_resolution_width = 1600;
    private final int camera_resolution_height = 1200;
    private final int camera_auto_exposure = 1;
    private final int camera_exposure = 10;
    private final double camera_gain = 0.0;
    private final double camera_brightness = 0.0;

    private final DoubleSubscriber latencySub;
    private final IntegerSubscriber fpsSub;
    private final DoubleArraySubscriber poseSub;
    private final IntegerArraySubscriber tidSub;

    private final DoubleArrayPublisher layoutSub;

    private double[] rawPoseData;
    private double rawLatency;
    private long[] rawIdData;

    public OV2311Camera(String name, Pose3d offset) {
        this.field = AbstractOdometry.getInstance().getField();

        this.name = name;
        this.offset = offset;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(this.name);

        NetworkTable configTable = table.getSubTable("config");
        configTable.getIntegerTopic("camera_id").publish().set(camera_id);
        configTable.getIntegerTopic("camera_resolution_width").publish().set(camera_resolution_width);
        configTable.getIntegerTopic("camera_resolution_height").publish().set(camera_resolution_height);
        configTable.getIntegerTopic("camera_auto_exposure").publish().set(camera_auto_exposure);
        configTable.getIntegerTopic("camera_exposure").publish().set(camera_exposure);
        configTable.getDoubleTopic("camera_gain").publish().set(camera_gain);
        configTable.getDoubleTopic("camera_brightness").publish().set(camera_brightness);
        configTable.getDoubleTopic("fiducial_size").publish().set(Field.FIDUCIAL_SIZE);
        layoutSub = configTable.getDoubleArrayTopic("fiducial_layout").publish();
        layoutSub.set(Field.getFiducialLayoutAsDoubleArray(Field.FIDUCIALS));
        configTable.getDoubleArrayTopic("camera_offset").publish()
            .set(new double[] {
                offset.getX(),
                offset.getY(),
                offset.getZ(),
                offset.getRotation().getX(),
                offset.getRotation().getY(),
                offset.getRotation().getZ(),
            });
        configTable.getDoubleArrayTopic("fiducial_poses").publish().set(Field.getFiducialPosesAsDoubleArray(Field.FIDUCIALS));

        NetworkTable outputTable = table.getSubTable("output");
        latencySub = outputTable.getDoubleTopic("latency").subscribe(0);
        fpsSub = outputTable.getIntegerTopic("fps").subscribe(0);
        poseSub = outputTable.getDoubleArrayTopic("pose")
            .subscribe(
                new double[] {},
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true));
        tidSub = outputTable.getIntegerArrayTopic("tids").subscribe(new long[] {});
    }

    public String getName() {
        return name;
    }

    private boolean hasData() {
        return rawPoseData.length > 0 &&
               rawIdData.length > 0;
    }

    private Pose3d poseFromArray(double[] rawData) {
        return new Pose3d(
            rawData[0], rawData[1], rawData[2],
            new Rotation3d(rawData[3], rawData[4], rawData[5]));
    }

    private Pose3d getRobotPose() {
        return poseFromArray(rawPoseData).transformBy(
            new Transform3d(offset.getTranslation(), offset.getRotation()).inverse());
    }

    private boolean isValidData(VisionData data) {
        for (long id : data.tids) {
            boolean found = false;
            for (Fiducial f : Field.FIDUCIALS) {
                if (f.getID() == id) {
                    found = true;
                    break;
                }
            }

            if (!found) {
                return false;
            }
        }

        if (Double.isNaN(data.robotPose.getX()) || data.robotPose.getX() < 0  || data.robotPose.getX() > Field.WIDTH) return false;
        if (Double.isNaN(data.robotPose.getY()) || data.robotPose.getY() < 0  || data.robotPose.getY() > Field.HEIGHT) return false;
        if (Double.isNaN(data.robotPose.getZ()) || data.robotPose.getZ() < -1 || data.robotPose.getZ() > 1) return false;

        return true;
    }

    public Optional<VisionData> getVisionData() {
        updateData();

        if (!hasData()) return Optional.empty();
        field.getObject(name).setPose(getRobotPose().toPose2d());

        double fpgaTime = latencySub.getLastChange() / 1_000_000.0;
        double timestamp = fpgaTime - Units.millisecondsToSeconds(rawLatency);

        LogPose3d.logPose3d("Vision/Robot Pose", getRobotPose());

        VisionData data = new VisionData(rawIdData, offset, getRobotPose(), timestamp);

        if (!isValidData(data)) return Optional.empty();

        return Optional.of(data);
    }

    public void setLayout(Fiducial[] layout) {
        layoutSub.set(Field.getFiducialLayoutAsDoubleArray(layout));
    }

    private void updateData() {
        rawLatency = latencySub.get();
        rawPoseData = poseSub.get();
        rawIdData = tidSub.get();
    }

    public long getFPS() {
        return fpsSub.get();
    }
}
