package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class Leds {
    RevBlinkinLedDriver led;

    final static RevBlinkinLedDriver.BlinkinPattern[] patterns = {
            RevBlinkinLedDriver.BlinkinPattern.BLUE,
            RevBlinkinLedDriver.BlinkinPattern.RED,
            RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE,
            RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE,
    };

    public Leds(Robot robot) {
        led = robot.led;
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    void setPattern(int index) {
        led.setPattern(patterns[index]);
    }
}
