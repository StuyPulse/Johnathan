package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.subsystems.leds.*;
import com.stuypulse.robot.subsystems.leds.instructions.LEDInstruction;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDSet extends InstantCommand {

    private LEDController controller;
    private LEDInstruction instruction;

    public LEDSet(LEDInstruction instruction) {
        this.controller = LEDController.getInstance();
        this.instruction = instruction;
    }

    @Override
    public void initialize() {
        controller.forceSetLED(instruction);
    }
}
