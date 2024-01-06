package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Robot {
    public DcMotor motorFR = null; // Front Right
    public DcMotor motorFL = null; // Front Left
    public DcMotor motorBR = null; // Back Right
    public DcMotor motorBL = null; // Back Left

    public DcMotorEx motorSlider; // Slider
    public Servo servoArm; // Elbow or Arm

    public Servo servoCR; // Claw Right
    public Servo servoCL; // Claw left
    public Servo servoDrone; // Drone
    public Telemetry telemetry;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    public WebcamName webcam;
    public SampleMecanumDrive sampleDrive;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        motorFL = hardwareMap.get(DcMotor.class, "front_left");
        motorFR = hardwareMap.get(DcMotor.class, "front_right");
        motorBL = hardwareMap.get(DcMotor.class, "back_left");
        motorBR = hardwareMap.get(DcMotor.class, "back_right");
        motorSlider = hardwareMap.get(DcMotorEx.class, "sliders");
        servoArm = hardwareMap.get(Servo.class, "arm");
        servoCL = hardwareMap.get(Servo.class, "claw_left");
        servoCR = hardwareMap.get(Servo.class, "claw_right");
        servoDrone = hardwareMap.get(Servo.class, "drone");
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        sampleDrive = new SampleMecanumDrive(hardwareMap);

        //servoArm.setPosition(0.2);
    }

}
