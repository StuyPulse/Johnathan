/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.notevision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractNoteVision extends SubsystemBase {

    /** SINGLETON **/
    private static final AbstractNoteVision instance;

    static {
        instance = new NoteVision();
    }

    public static AbstractNoteVision getInstance() {
        return instance;
    }
    
    public abstract boolean hasNoteData();

    public abstract Translation2d getEstimatedNotePose();

}
