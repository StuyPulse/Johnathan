package com.stuypulse.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public interface LEDInstruction {
    void setLED(AddressableLEDBuffer ledsBuffer);
}
