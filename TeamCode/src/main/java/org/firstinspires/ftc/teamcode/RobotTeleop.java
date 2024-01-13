package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SciRavens-TeleOp")
public class RobotTeleop extends LinearOpMode {
    public Robot robot;
    public DriveTrain DT;
    public DroneLauncher DL;
    public Slider slider;
    public Arm arm;
    public Claw left_claw, right_claw;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        DT = new DriveTrain(robot, gamepad1);
        DL = new DroneLauncher(robot, gamepad1);
        slider = new Slider(robot, gamepad2);
        arm = new Arm(robot, gamepad2);
        left_claw = new Claw(robot.servoCR, robot.claw_left_close, robot.claw_left_open);
        //left_claw = new Claw(robot.servoCR, 0.75, 1);
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        right_claw = new Claw(robot.servoCL, robot.claw_right_close, robot.claw_right_open);
        waitForStart();
        robot.led.setPattern(pattern);
        while(opModeIsActive()) {
            DT.drive();
            DL.launchDrone();
            slider.operate();
            arm.operate();
            claws_operate();
        }
    }
    private void claws_operate() {
        if (gamepad2.right_trigger > 0.9) {
            right_claw.open();
        } else {
            right_claw.close();
        }
        if (gamepad2.left_trigger > 0.9) {
            left_claw.open();
        } else {
            left_claw.close();
        }
    }
}
