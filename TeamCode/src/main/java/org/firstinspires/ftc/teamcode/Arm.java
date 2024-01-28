package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Arm {
    private Robot robot;
    private Gamepad gamepad;
    static private double pos_pixel  = 0.1;
    static private double pos_folded  = 1.0;
    static private double pos_backdrop  = 0.6;

    static private double pos_whitepixel = 0.215;
    public Arm(Robot robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
        arm_pixel();
    }

    public void arm_pixel()
    {
        robot.servoArm.setPosition(pos_pixel);
    }

    public void arm_fold()
    {
        robot.servoArm.setPosition(pos_folded);
    }

    public void arm_backdrop() {
        robot.servoArm.setPosition(pos_backdrop);
    }

    public void arm_whitepixel() {
        robot.servoArm.setPosition(pos_whitepixel);
    }
    public void operate()
    {
        if (gamepad.x) {
            arm_pixel();
        } else if (gamepad.y) {
            arm_backdrop();
        } else if (gamepad.b) {
            arm_fold();
        } else if (gamepad.right_stick_y != 0) {
            //robot.servoArm.setPosition((1.0 - gamepad.right_stick_y ) % 1.0);
        }
    }
}
