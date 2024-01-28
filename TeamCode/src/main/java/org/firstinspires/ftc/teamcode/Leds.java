package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class Leds {
    RevBlinkinLedDriver led;

    // private int cur = 0;
    final static RevBlinkinLedDriver.BlinkinPattern[] patterns = {
            RevBlinkinLedDriver.BlinkinPattern.BLUE,
            RevBlinkinLedDriver.BlinkinPattern.RED,
            RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE,
            RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE,
            RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE,
            RevBlinkinLedDriver.BlinkinPattern.CONFETTI,
            RevBlinkinLedDriver.BlinkinPattern.WHITE,
            RevBlinkinLedDriver.BlinkinPattern.GREEN,
            RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE,
            RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE,
            RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE,
            RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE,
            RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE,
            RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE,
            RevBlinkinLedDriver.BlinkinPattern.CP1_END_TO_END_BLEND_TO_BLACK,
            RevBlinkinLedDriver.BlinkinPattern.HOT_PINK,
            RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_1_ON_2,
            RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_2_ON_1,
            RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT,
            RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE,
            RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY,
            RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED,
            RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE,
            RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES
    };

    public Leds(Robot robot) {
        led = robot.led;
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    void setPattern(int index) {
        led.setPattern(patterns[index]);
    }
}
