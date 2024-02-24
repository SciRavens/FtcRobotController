package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Far-Blue-Autonomous")
public class FarBlueAutonomous extends LinearOpMode {
    public Robot robot;
    public SampleMecanumDrive drive;
    public Slider slider;
    public Arm arm;
    public Claw left_claw, right_claw;
    public AprilTag tag;
    public TgeDetection tge;
    String curAlliance = "red";
    public int zone = -1;
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

        //arm.arm_backdrop();
        arm.arm_fold();
        right_claw.open();
        left_claw.open();

        //tag = new AprilTag(robot);
        tge = new TgeDetection(robot, "blue");
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
        right_claw.close();   //init code
        left_claw.close();
        zone = tge.getZone();
        telemetry.addData("Zone number:", zone);
        telemetry.update();
        tge.stop();
        leds.setPattern(0);

        if(opModeIsActive()) {
            //zone = 2;
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

    private void buildBlueZone1Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajBlueZone1 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    sleep(500);
                })
                .addTemporalMarker(() -> {
                    arm.arm_pixel();
                })
                .waitSeconds(0.5)
                .forward(22)
                .turn(Math.toRadians(50))
                .forward(4)
                .addTemporalMarker(() -> {
                    right_claw.open();
                    sleep(500);
                    arm.arm_fold();  //places purple pixel
                })
                .back(3)
                .turn(Math.toRadians(-50))
                .forward(30)
                .turn(Math.toRadians(90))
                .forward(72) //goes through middle fence
                .strafeLeft(37)
                .forward(15.235)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    arm.arm_backdrop();
                    slider.auton();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    left_claw.open();
                    sleep(500);
                    arm.arm_fold(); //places pixel on the backdrop
                    sleep(500);
                })
                .waitSeconds(1)
                .back(15)
                .strafeRight(32)
                .turn(Math.toRadians(180))
                .back(23) //parks in the middle
                .addTemporalMarker(() -> {
                    arm.arm_fold();
                })
                .build();

    }

    private void buildBlueZone2Trajectory() {
        //Pose2d startPose = new Pose2d(-35.5, 64, Math.toRadians(270));
        Pose2d startPose = new Pose2d(0, 0, 0);
        //drive.setPoseEstimate(startPose);
        trajBlueZone2 = drive.trajectorySequenceBuilder(startPose)
                .forward(50.5)
                .turn(Math.toRadians(185))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    arm.arm_pixel();
                    sleep(500);
                    right_claw.open();
                    sleep(500);
                    arm.arm_backdrop(); //places purple pixel
                    sleep(500);
                })
                .back(6.5)
                .turn(Math.toRadians(-93))
                .waitSeconds(0.5)
                .forward(70) //goes through the middle fence
                .strafeLeft(30.65) // 28.55 old
                .waitSeconds(0.5)
                .forward(19) //goes towards back drop
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(1000);
                    right_claw.open();
                    left_claw.open();
                    sleep(500);
                    arm.arm_fold(); //places pixel on the back drop
                    sleep(500);
                })
                .back(10)
                .waitSeconds(0.5)
                .strafeRight(21.5)
                .waitSeconds(0.5)
                .turn(Math.toRadians(180))
                .waitSeconds(0.5)
                .back(15) //parks
                .build();
    }

    private void buildBlueZone3Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajBlueZone3 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    sleep(500);
                })
                .addTemporalMarker(() -> {
                    arm.arm_pixel();
                })
                .waitSeconds(0.5)
                .forward(23)
                .turn(Math.toRadians(-50))
                .forward(1)
                .addTemporalMarker(() -> {
                    right_claw.open();
                    sleep(500);
                    arm.arm_fold(); //places pixel
                })
                .back(2)
                .turn(Math.toRadians(50))
                .forward(32.5)
                .turn(Math.toRadians(90))
                .forward(72)  //goes through the middle fence
                .strafeLeft(25.5)
                .forward(14.56)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    arm.arm_backdrop();
                    slider.auton();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    left_claw.open(); //places pixel on the backdrop
                })
                .waitSeconds(0.5)
                .back(15)
                .strafeRight(17)
                .turn(Math.toRadians(180))
                .back(23) //parks
                .addTemporalMarker(() -> {
                    arm.arm_fold();
                })
                .build();
    }

}

