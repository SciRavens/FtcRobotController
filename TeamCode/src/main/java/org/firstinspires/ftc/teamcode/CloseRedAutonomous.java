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
        leds.setPattern(1);

        // Fold and open the claws for placing the pixels
        arm.arm_fold();
        right_claw.close();
        left_claw.close();
        sleep(500);
        right_claw.open();
        left_claw.open();

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
        arm.arm_pixel();
        sleep(500);

        zone = tge.getZone();
        telemetry.addData("Zone number:", zone);
        telemetry.update();

        if(opModeIsActive()) {
            //zone = 2;
            switch(zone) {
                case 1:
                    // Zone1 Auton
                    robot.sampleDrive.followTrajectorySequence(trajRedZone1);;
                    break;
                case 2:
                    // Zone2 Auton
                    robot.sampleDrive.followTrajectorySequence(trajRedZone2);;
                    break;
                case 3:
                    // Zone3 Auton
                    robot.sampleDrive.followTrajectorySequence(trajRedZone3);;
                    break;
            }
        }
        slider.fold();
        right_claw.close();
        left_claw.close();
        leds.setPattern(10);
        sleep(1000);
    }

    // Build Zone1 trajectory
    private void buildRedZone1Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajRedZone1 = drive.trajectorySequenceBuilder(startPose)
                .forward(25)
                .turn(Math.toRadians(55))
                .forward(0.89878) // now at the Zone1
                .waitSeconds(1)
                // Leave purple pixel
                .addTemporalMarker(() -> {
                    left_claw.open();
                    sleep(500);
                    arm.arm_backdrop();
                    sleep(500);
                })
                .waitSeconds(1)
                // Go to the backdrop
                .turn(Math.toRadians(-145))
                .forward(22.5)
                .waitSeconds(0.5)
                .strafeLeft(10)
                // raise the slider
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(500);
                })
                .waitSeconds(0.5)
                .forward(19) // now at the backdrop
                // Drop the yellow pixel on the backdrop
                .addTemporalMarker(() -> {
                    right_claw.open();
                    sleep(500);
                    arm.arm_fold();
                    sleep(500);
                })
                // Go to the parking spot
                .back(4)
                .waitSeconds(0.5)
                .strafeRight(32)
                .turn(Math.toRadians(180))
                .back(10) // now at the parking
                .build();
    }

    // Build Zone2 trajectory
    private void buildRedZone2Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajRedZone2 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .forward(28)    // now at the zone2
                // Leave the purple pixel
                .addTemporalMarker(() -> {
                    left_claw.open();
                    sleep(500);
                    arm.arm_backdrop();
                    sleep(500);
                })
                .waitSeconds(1)
                // Go to the backdrop
                .forward(1.05)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(500);
                })
                .waitSeconds(0.5)
                .forward(41)    // Now at the backdrop

                // Drop the yellow pixel on the backdrop
                .addTemporalMarker(() -> {
                    right_claw.open();
                    sleep(500);
                    arm.arm_fold();
                    sleep(500);
                })

                // Go to the parking spot
                .back(4)
                .waitSeconds(0.5)
                .strafeRight(28)
                .turn(Math.toRadians(180))
                .back(7)    //now at the parking
                .build();
    }

    // Build Zone3 trajectory
    private void buildRedZone3Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajRedZone3 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .strafeRight(12.5)
                .forward(22.5)      // now at the zone3

                // leave the purple pixel
                .addTemporalMarker(() -> {
                    left_claw.open();
                    sleep(500);
                    arm.arm_backdrop();
                    sleep(500);
                })
                .waitSeconds(1)

                //Go to the backdrop
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(500);
                })
                .waitSeconds(0.5)
                .forward(29.77) // now at the backdrop

                // Drop the yellow pixel on the backdrop
                .addTemporalMarker(() -> {
                    right_claw.open();
                    sleep(500);
                    arm.arm_fold();
                    sleep(500);
                })
                // Go to the parking
                .back(4)
                .waitSeconds(0.5)
                .strafeRight(24)
                .turn(Math.toRadians(180))
                .back(10)   // now at the parking
                .build();
    }
}

