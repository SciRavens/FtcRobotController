package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Close-Blue-Autonomous")
public class CloseBlueAutonomous extends LinearOpMode {
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
        //tag = new AprilTag(robot);
        tge = new TgeDetection(robot, "blue");
        leds = new Leds(robot);
        leds.setPattern(0); // Set LED colors to Blue
        arm.arm_fold();

        // Fold and open the claws for placing the pixels
        arm.arm_fold();
        right_claw.close();
        left_claw.close();
        sleep(500);
        right_claw.open();
        left_claw.open();

        // Build the Trajectories in the Init mode so that we save time
        // during the run time.
        buildBlueZone1Trajectory();
        buildBlueZone2Trajectory();
        buildBlueZone3Trajectory();

        // Detect the Zone while we are in the init mode
        while(tge.getZone() == -1) {
            telemetry.addData("CAMERA INIT:", tge.getZone());
            telemetry.update();
            sleep(100);
        }
        zone = tge.getZone();
        telemetry.addData("INIT Zone number:", zone);
        telemetry.update();


        // Wait for Start Button to be pressed
        waitForStart();
        if(isStopRequested()) {
            return;
        }

        // Start the auton code
        // Close both claws first
        left_claw.close();
        right_claw.close();
        sleep(500);

        // Bring the arm to the floor to push the pixels the zones
        arm.arm_pixel();

        // Get the zone 5 times to avoid any random zone values
        for (int i = 0; i < 5; i++) {
            zone = tge.getZone();
            telemetry.addData("Zone number:", zone);
            telemetry.update();
            sleep(100);
        }


        if(opModeIsActive()) {
            //zone = 3;
            switch(zone) {
                case 1:
                    // Start the zone1 auton code
                    robot.sampleDrive.followTrajectorySequence(trajBlueZone1);;
                    break;
                case 2:
                    // Start the zone2 auton code
                    robot.sampleDrive.followTrajectorySequence(trajBlueZone2);;
                    break;
                case 3:
                    // Start the zone3 auton code
                    robot.sampleDrive.followTrajectorySequence(trajBlueZone3);;
                    break;
            }
        }
        slider.fold(); // bring the slider down
        right_claw.close();
        left_claw.close();
        leds.setPattern(10);
        sleep(1000);
    }

    // Build Zone1 Trajectory
    private void buildBlueZone1Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajBlueZone1 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .strafeLeft(13.25)
                .forward(20)    // Go to the zone1
                .addTemporalMarker(() -> {
                    // Leave the purple pixel at zone1
                    right_claw.open();
                    sleep(500);
                    // raise the arm to backrop position to drop pixel later
                    arm.arm_backdrop();
                    sleep(500);
                })
                .waitSeconds(1)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    // Raise the sliders to close to the backdrop
                    slider.auton();
                    sleep(1000); // given enough time
                })
                .waitSeconds(0.5)
                .forward(29.25) // Now at the Backdrop
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    // Open the claw to drop the yellow pixel
                    left_claw.open();
                    sleep(500);
                    arm.arm_fold();    // Fold the arm away from the backdrop
                    sleep(500);
                })
                // Now start parking
                .back(10)
                .waitSeconds(0.5)
                .turn(Math.toRadians(180))
                .waitSeconds(0.5)
                .strafeRight(20)
                .waitSeconds(0.5)
                .back(16) // Parked
                .build();
    }

    // Build Zone2 Trajectory
    private void buildBlueZone2Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajBlueZone2 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .forward(27) // Go to the zone2
                .addTemporalMarker(() -> {
                    // Open the right claw to leave the purple pixel
                    right_claw.open();
                    sleep(500);
                    arm.arm_backdrop(); // raise the arm to leave the pixel
                    sleep(500);
                })
                .waitSeconds(1)
                .back(1.25)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    slider.auton();         // slider close to the backdrop
                    sleep(1000);
                })
                .waitSeconds(0.5)
                .forward(41.25) // Now at the backdrop
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    left_claw.open();       // open the claw to drop yello pixel
                    sleep(500);
                    arm.arm_fold();     // fold the arm
                    sleep(500);
                })
                .back(10)    // back away from the backdrop
                .waitSeconds(0.5)
                .turn(Math.toRadians(180))  // turn backwards
                .waitSeconds(0.5)
                .strafeRight(25)  // go to the parking spot
                .waitSeconds(0.5)
                .back(16)            // park
                .build();
    }

    // Build Zone3 trajectory
    private void buildBlueZone3Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajBlueZone3 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .forward(26)
                .turn(Math.toRadians(-55))
                .forward(1.27)      // go to the Zone3
                .waitSeconds(1)
                // Drop the pixel and raise the arm to backdrop
                .addTemporalMarker(() -> {
                    right_claw.open();
                    sleep(500);
                    arm.arm_backdrop();
                    sleep(500);
                })
                .waitSeconds(1)
                // Go to the backdrop
                .turn(Math.toRadians(145))
                .forward(21.5)
                .waitSeconds(0.5)
                .strafeRight(8)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(1000);
                })
                .waitSeconds(0.5)
                .forward(20.5)  // now at the backdrop
                .waitSeconds(1)
                // drop the pixel on the backdrop
                .addTemporalMarker(() -> {
                    left_claw.open();
                    sleep(500);
                    arm.arm_fold();
                    sleep(500);
                })
                // Go to the parking spot
                .back(10)
                .waitSeconds(0.5)
                .turn(Math.toRadians(180))
                .waitSeconds(0.5)
                .strafeRight(37)
                .waitSeconds(0.5)
                .back(16) // now at the parking
                .build();
    }

}

