package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Far-Red-Autonomous")
public class FarRedAutonomous extends LinearOpMode {
    public Robot robot;
    public SampleMecanumDrive drive;
    public Slider slider;
    public Arm arm;
    public Claw left_claw, right_claw;
    public AprilTag tag;
    public TgeDetection tge;
    String curAlliance = "red";
    public int zone = 2;
    TrajectorySequence trajBlueZone1;
    TrajectorySequence trajBlueZone2;
    TrajectorySequence trajBlueZone3;

    Leds leds;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        drive = robot.sampleDrive;
        slider = new Slider(robot, gamepad2);
        arm = new Arm(robot, gamepad2);

        left_claw = new Claw(robot.servoCR, robot.claw_left_close, robot.claw_left_open);

        right_claw = new Claw(robot.servoCL, robot.claw_right_close, robot.claw_right_open);
        leds = new Leds(robot);

        //tag = new AprilTag(robot);
        tge = new TgeDetection(robot, "red");

        buildBlueZone1Trajectory();
        buildBlueZone2Trajectory();
        buildBlueZone3Trajectory();

        while(tge.getZone() == -1) {
            telemetry.addData("CAMERA INIT:", zone);
            telemetry.update();
            sleep(100);
        }
        zone = tge.getZone();
        telemetry.addData("INIT Zone number:", zone);
        telemetry.update();

        waitForStart();
        if(isStopRequested()) {
            return;
        }
        zone = tge.getZone();
        telemetry.addData("Zone number:", zone);
        telemetry.update();

        leds.setPattern(1);


        if(opModeIsActive()) {
            //zone = 3;
            switch(zone) {
                case 1:
                    robot.sampleDrive.followTrajectorySequence(trajBlueZone1);;
                    break;
                case 2:
                    robot.sampleDrive.followTrajectorySequence(trajBlueZone2);;
                    break;
                case 3:
                    robot.sampleDrive.followTrajectorySequence(trajBlueZone3);;
                    break;
            }
        }

    }

    private void buildBlueZone3Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajBlueZone3 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    right_claw.close();
                    left_claw.close();
                    sleep(500);
                })
                .waitSeconds(1)
                .forward(22)
                .turn(Math.toRadians(-50))
                .forward(4)
                .addTemporalMarker(() -> {
                    left_claw.open();
                    sleep(500);
                    arm.arm_fold();
                    sleep(500);
                })
                .back(4)
                .turn(Math.toRadians(50))
                .waitSeconds(1)
                .back(19)
                .addTemporalMarker(() -> {
                    arm.arm_backdrop();
                    sleep(500);
                })
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .waitSeconds(2)
                .forward(70)
                .strafeLeft(22)
                .forward(19)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(500);
                    right_claw.open();
                    sleep(1000);
                    arm.arm_fold();
                    sleep(500);
                })
                .back(4)
                .strafeRight(21)
                .forward(10)
                .build();
    }

    private void buildBlueZone2Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajBlueZone2 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    right_claw.close();
                    left_claw.close();
                    sleep(500);
                })
                .waitSeconds(1)
                .forward(27)
                .addTemporalMarker(() -> {
                    left_claw.open();
                    sleep(500);
                    arm.arm_backdrop();
                    sleep(500);
                    left_claw.close();
                    sleep(500);
                })
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .forward(87)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(500);
                    right_claw.open();
                    sleep(1000);
                    arm.arm_fold();
                    sleep(500);
                })
                .back(4)
                .strafeRight(25)
                .forward(10)
                .build();

    }

    private void buildBlueZone1Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajBlueZone1 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    right_claw.close();
                    left_claw.close();
                    sleep(500);
                })
                .waitSeconds(7)
                .forward(25)
                .turn(Math.toRadians(55))
                .forward(1)
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    left_claw.open();
                    sleep(500);
                    arm.arm_backdrop();
                    sleep(500);
                })
                .waitSeconds(1)
                .turn(Math.toRadians(-145))
                .forward(89)
                .strafeLeft(10)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(500);
                    right_claw.open();
                    sleep(1000);
                    arm.arm_fold();
                    sleep(500);
                    sleep(15000);
                })
                .back(4)
                .strafeRight(31.5)
                .forward(10)
                .build();
    }

}

