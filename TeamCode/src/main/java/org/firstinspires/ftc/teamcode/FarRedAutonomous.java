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
    public int zone = -1;
    TrajectorySequence trajZone1;
    TrajectorySequence trajZone2;
    TrajectorySequence trajZone3;
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

        //arm.arm_backdrop();
        arm.arm_fold();
        right_claw.open();
        left_claw.open();

        //tag = new AprilTag(robot);
        tge = new TgeDetection(robot, "blue");
        buildRedZone1Trajectory();
        buildRedZone2Trajectory();
        buildRedZone3Trajectory();
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
        right_claw.close();
        left_claw.close();
        zone = tge.getZone();
        telemetry.addData("Zone number:", zone);
        telemetry.update();
        tge.stop();
        leds.setPattern(0);

        if(opModeIsActive()) {
            //zone = 1;
            switch(zone) {
                case 1:
                    robot.sampleDrive.followTrajectorySequence(trajZone1);;
                    break;
                case 2:
                    robot.sampleDrive.followTrajectorySequence(trajZone2);;
                    break;
                case 3:
                    robot.sampleDrive.followTrajectorySequence(trajZone3);;
                    break;
            }
        }

    }

    private void buildRedZone3Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajZone3 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    sleep(500);
                })
                .addTemporalMarker(() -> {
                    arm.arm_pixel();
                })
                .waitSeconds(0.5)
                .forward(22)
                .turn(Math.toRadians(-50))
                .forward(4)
                .addTemporalMarker(() -> {
                    left_claw.open();
                    sleep(500);
                    arm.arm_fold();
                })
                .back(3)
                .turn(Math.toRadians(50))
                .forward(30)
                .turn(Math.toRadians(-90))
                .forward(72)
                .strafeRight(30)//p
                .forward(16.15)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    arm.arm_backdrop();
                    slider.auton();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    right_claw.open();
                })
                .waitSeconds(0.5)
                .back(15)
                .strafeLeft(29.5)
                .turn(Math.toRadians(-180))
                .back(20)
                .build();


    }

    private void buildRedZone2Trajectory() {
        //Pose2d startPose = new Pose2d(-35.5, 64, Math.toRadians(270));
        Pose2d startPose = new Pose2d(0, 0, 0);
        //drive.setPoseEstimate(startPose);
        trajZone2 = drive.trajectorySequenceBuilder(startPose)
                .forward(53.007)
                .turn(Math.toRadians(-185))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> { // drops purple pixel
                    arm.arm_pixel();
                    sleep(500);
                    left_claw.open();
                    sleep(500);
                    arm.arm_backdrop();
                    sleep(500);
                })
                .back(5)
                .turn(Math.toRadians(93))
                .waitSeconds(0.5)
                .forward(70)
                .strafeRight(25)
                .waitSeconds(0.5)
                .forward(17.5) // go to the backdrop
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(1000);
                    right_claw.open();
                    sleep(500);
                    arm.arm_fold();
                    sleep(500);
                })
                .back(10)
                .waitSeconds(1)
                .strafeLeft(22)
                .waitSeconds(1)
                .turn(Math.toRadians(-180)) // 3.14159265359
                .waitSeconds(0.5)
                .back(15)
                .build();
    }

    private void buildBlueZone2longTrajectory() {
        //Pose2d startPose = new Pose2d(-35.5, 64, Math.toRadians(270));
        Pose2d startPose = new Pose2d(0, 0, 0);
        //drive.setPoseEstimate(startPose);
        trajZone2 = drive.trajectorySequenceBuilder(startPose)
                .forward(49.007)
                .turn(Math.toRadians(185))
                //.back(3)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> { // drops purple pixel
                    arm.arm_pixel();
                    sleep(500);
                    right_claw.open();
                    sleep(500);
                    arm.arm_backdrop();
                    sleep(500);
                })

                .addTemporalMarker(() -> {
                    right_claw.close();
                    sleep(1000);
                    arm.arm_backdrop();
                    sleep(500);
                })
                .waitSeconds(10)
                .turn(Math.toRadians(180))
                .waitSeconds(0.5)
                .forward(92)
                .strafeLeft(41)
                .waitSeconds(0.5)
                .forward(5)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    right_claw.open();
                    left_claw.open();
                    sleep(500);
                    left_claw.close();
                    right_claw.close();
                    sleep(500);
                    arm.arm_fold();
                    sleep(500);
                })
                .back(12)
                .strafeRight(26)
                .turn(Math.toRadians(180))
                .back(25)
                .build();
    }

    private void buildRedZone1Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajZone1 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    sleep(500);
                })
                .addTemporalMarker(() -> {
                    arm.arm_pixel();
                })
                .waitSeconds(0.5)
                .forward(23)
                .turn(Math.toRadians(50))
                .forward(1)
                .addTemporalMarker(() -> {
                    left_claw.open();
                    sleep(500);
                    arm.arm_fold();
                })
                .back(2)
                .turn(Math.toRadians(-50))
                .forward(32.5)
                .turn(Math.toRadians(-90))
                .forward(72)
                .strafeRight(18.3)
                .forward(16)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    arm.arm_backdrop();
                    slider.auton();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    right_claw.open();
                })
                .waitSeconds(0.5)
                .back(15)
                .strafeLeft(16)
                .turn(Math.toRadians(-180))
                .back(19)
                .build();
    }

}

