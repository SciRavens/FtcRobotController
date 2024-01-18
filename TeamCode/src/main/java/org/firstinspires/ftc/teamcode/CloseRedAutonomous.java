package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Close-Red-Autonomous")
public class CloseRedAutonomous extends LinearOpMode {
    public Robot robot;
    public SampleMecanumDrive drive;
    public Slider slider;
    public Arm arm;
    public Claw left_claw, right_claw;

    public AprilTag tag;
    public TgeDetection tge;
    String curAlliance = "red";
    public int zone = 2;
    TrajectorySequence trajRedZone1;
    TrajectorySequence trajRedZone2;
    TrajectorySequence trajRedZone3;

    Leds leds;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        drive = robot.sampleDrive;
        slider = new Slider(robot, gamepad2);
        arm = new Arm(robot, gamepad2);
        left_claw = new Claw(robot.servoCR, robot.claw_left_close, robot.claw_left_open);
        right_claw = new Claw(robot.servoCL, robot.claw_right_close, robot.claw_right_open);
        //tag = new AprilTag(robot);
        tge = new TgeDetection(robot, "red");
        leds = new Leds(robot);

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
        zone = tge.getZone();
        telemetry.addData("Zone number:", zone);
        telemetry.update();

        leds.setPattern(1);
        if(opModeIsActive()) {
            //zone = 2;
            switch(zone) {
                case 1:
                    robot.sampleDrive.followTrajectorySequence(trajRedZone1);;
                    break;
                case 2:
                    robot.sampleDrive.followTrajectorySequence(trajRedZone2);;
                    break;
                case 3:
                    robot.sampleDrive.followTrajectorySequence(trajRedZone3);;
                    break;
            }
        }

    }
// change the order of tghe functions as it is 3,2,1 instead of numercal order; God bless America
    private void buildRedZone3Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajRedZone3 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    right_claw.close();
                    left_claw.close();
                    sleep(500);
                    // sleep(5000);
                })
                .waitSeconds(1)
                .strafeRight(12.5)
                .forward(22.5)
                .addTemporalMarker(() -> {
                    left_claw.open();
                    sleep(500);
                    arm.arm_backdrop();
                    sleep(500);
                })
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .forward(29.77)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(500);
                    right_claw.open();
                    sleep(1000);
                    arm.arm_fold();
                    sleep(500);
                })
                .back(4)
                .strafeRight(19)
                .forward(10)
                .build();
    }

    private void buildRedZone2Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajRedZone2 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    right_claw.close();
                    left_claw.close();
                    sleep(500);
                })
                .waitSeconds(1)
                .forward(28)
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
                .forward(39.5)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(500);
                    right_claw.open();
                    sleep(1000);
                    arm.arm_fold();
                    sleep(500);
                })
                .back(4)
                .strafeRight(23)
                .forward(10)
                .build();
    }

    private void buildRedZone1Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajRedZone1 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    right_claw.close();
                    left_claw.close();
                    sleep(500);
                })
                .waitSeconds(1)
                .forward(25)
                .turn(Math.toRadians(55))
                .forward(0.89878)
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    left_claw.open();
                    sleep(500);
                    arm.arm_backdrop();
                    sleep(500);
                })
                .waitSeconds(1)
                .turn(Math.toRadians(-145))
                .forward(22.5)
                .strafeLeft(10)
                .forward(18)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(500);
                    right_claw.open();
                    sleep(1000);
                    arm.arm_fold();
                    sleep(500);
                })
                .back(4)
                .strafeRight(32)
                .forward(10)
                .build();
    }

}

