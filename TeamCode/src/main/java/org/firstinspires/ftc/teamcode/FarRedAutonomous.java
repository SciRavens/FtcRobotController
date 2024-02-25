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
        leds.setPattern(1);

        //arm.arm_backdrop(); //before init program:
        arm.arm_fold();
        right_claw.close();
        left_claw.close();
        sleep(500);
        right_claw.open();
        left_claw.open();

        //tag = new AprilTag(robot);
        tge = new TgeDetection(robot, "red");

        // Build trajectories
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

        //
        waitForStart();
        if(isStopRequested()) {
            return;
        }
        right_claw.close();  //init programs
        left_claw.close();
        zone = tge.getZone();
        telemetry.addData("Zone number:", zone);
        telemetry.update();
        tge.stop();

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
        slider.fold();
        right_claw.close();
        left_claw.close();
        leds.setPattern(10);
        sleep(1000);
    }


    // Build Zone1 Trajectory
    private void buildRedZone1Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajZone1 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    arm.arm_pixel();
                    sleep(500);
                })
                .waitSeconds(0.5)
                // Go to the zone1
                .forward(23)
                .turn(Math.toRadians(50))
                .forward(1)     // now at the zone1
                // drop the purple pixel
                .addTemporalMarker(() -> {
                    left_claw.open(); //places purple pixel
                    sleep(500);
                    arm.arm_backdrop();
                    sleep(500);
                })
                // Now position near the gate
                .back(2)
                .turn(Math.toRadians(-50))
                .forward(32.5)
                .turn(Math.toRadians(-90))

                // Go through the gate
                .forward(72)
                .waitSeconds(0.5)
                .strafeRight(19.75 )
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(1000);
                })
                .waitSeconds(1)
                .forward(14) // go to the backdrop
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    right_claw.open(); //places pixel on the backdrop
                    sleep(500);
                    arm.arm_fold();
                    sleep(500);
                })
                .waitSeconds(0.5)
                // Go to the parking
                .back(15)
                .waitSeconds(0.5)
                .strafeLeft(16)
                .turn(Math.toRadians(-180))
                .back(19) //parks
                .build();
    }

    // Zone2 trajectory
    private void buildRedZone2Trajectory() {
        //Pose2d startPose = new Pose2d(-35.5, 64, Math.toRadians(270));
        Pose2d startPose = new Pose2d(0, 0, 0);
        //drive.setPoseEstimate(startPose);
        trajZone2 = drive.trajectorySequenceBuilder(startPose)
                .forward(53.007)
                .turn(Math.toRadians(-185)) // Now at the zone2
                .waitSeconds(0.5)

                // Drop the purple pixel
                .addTemporalMarker(() -> {
                    arm.arm_pixel();
                    sleep(500);
                    left_claw.open();
                    sleep(500);
                    arm.arm_backdrop();
                    sleep(500);
                })

                // position near the gate
                .back(5)
                .turn(Math.toRadians(93))
                .waitSeconds(0.5)

                // go through the gate
                .forward(70)
                .waitSeconds(0.5)
                .strafeRight(25.45)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(1000);
                })
                .waitSeconds(0.5)
                .forward(17.65) // go to the backdrop
                .waitSeconds(0.5)

                // drop the yellow pixel on the backdrop
                .addTemporalMarker(() -> {
                    right_claw.open(); //places pixel on the back drop
                    sleep(500);
                    arm.arm_fold();
                    sleep(500);
                })

                // Go to the parking
                .back(10)
                .waitSeconds(1)
                .strafeLeft(23)
                .waitSeconds(1)
                .turn(Math.toRadians(-180))
                .waitSeconds(0.5)
                .back(15) //now at the parking
                .build();
    }


    // Zone3 trajectory
    private void buildRedZone3Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajZone3 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    arm.arm_pixel();
                    sleep(500);
                })
                .waitSeconds(0.5)
                .forward(22)
                .turn(Math.toRadians(-50))
                .forward(4)  // now at the zone3

                // Drop the purple pixel
                .addTemporalMarker(() -> {
                    left_claw.open(); //places purple pixel
                    sleep(500);
                    arm.arm_backdrop();
                    sleep(500);
                })

                // Position near the gate
                .back(3)
                .turn(Math.toRadians(50))
                .forward(30)
                .turn(Math.toRadians(-90))

                // Go through the gate
                .forward(72)
                .waitSeconds(0.5)
                .strafeRight(30.5)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(1000);
                })
                .waitSeconds(0.5)
                .forward(14)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    right_claw.open(); //places yellow pixel on the back drop
                    sleep(1000);
                    arm.arm_fold();
                    sleep(500);
                })
                .waitSeconds(0.5)
                // Go to the parking
                .back(15)
                .waitSeconds(1)
                .strafeLeft(29.5)
                .turn(Math.toRadians(-180))
                .back(20) // now at the parking
                .build();


    }
}

