package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "SciRavens-Autonomous")
public class RobotAutonomous extends LinearOpMode {
    public Robot robot;
    public SampleMecanumDrive drive;
    public Slider slider;
    public Arm arm;
    public Claw claw;

    public AprilTag tag;
    public TgeDetection tge;
    String curAlliance = "red";
    public int zone = 2;
    TrajectorySequence trajBlueZone1;
    TrajectorySequence trajBlueZone2;
    TrajectorySequence trajBlueZone3;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        drive = robot.sampleDrive;
        slider = new Slider(robot, gamepad2);
        arm = new Arm(robot, gamepad2);
        claw = new Claw(robot, gamepad2);
        //tag = new AprilTag(robot);
        tge = new TgeDetection(robot);

        buildBlueZone1Trajectory();
        buildBlueZone2Trajectory();
        buildBlueZone3Trajectory();

        waitForStart();

        if(isStopRequested()) {
            return;
        }
            // Detect the zone
        for (int i = 0; i < 10; i++) {
            zone = tge.getZone();
            sleep(100);
            telemetry.addData("Zone number:", zone);
            telemetry.update();
        }

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

    private void buildBlueZone1Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajBlueZone1 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    claw.close_claw_right();
                    claw.close_claw_left();
                    sleep(500);
                })
                .waitSeconds(1)
                .strafeLeft(12.5)
                .forward(20)
                .addTemporalMarker(() -> {
                    claw.open_claw_right();
                    sleep(500);
                    arm.arm_backdrop();
                    sleep(500);
                })
                .waitSeconds(1)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .forward(27)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(500);
                    claw.open_claw_left();
                    sleep(1000);
                    arm.arm_fold();
                    sleep(500);
                })
                .back(4)
                .strafeLeft(19)
                .forward(10)
                .build();
    }

    private void buildBlueZone2Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajBlueZone2 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    claw.close_claw_right();
                    claw.close_claw_left();
                    sleep(500);
                })
                .waitSeconds(1)
                .forward(27)
                .addTemporalMarker(() -> {
                    claw.open_claw_right();
                    sleep(500);
                    arm.arm_backdrop();
                    sleep(500);
                    claw.close_claw_right();
                    sleep(500);
                })
                .waitSeconds(1)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .forward(39)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(500);
                    claw.open_claw_left();
                    sleep(1000);
                    arm.arm_fold();
                    sleep(500);
                })
                .back(4)
                .strafeLeft(25)
                .forward(10)
                .build();
    }

    private void buildBlueZone3Trajectory() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        trajBlueZone3 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    claw.close_claw_right();
                    claw.close_claw_left();
                    sleep(500);
                })
                .waitSeconds(1)
                .forward(26)
                .turn(Math.toRadians(-55))
                .forward(3)
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    claw.open_claw_right();
                    sleep(500);
                    arm.arm_backdrop();
                    sleep(500);
                })
                .waitSeconds(1)
                .turn(Math.toRadians(145))
                .forward(21)
                .strafeRight(8)
                .forward(20)
                .addTemporalMarker(() -> {
                    slider.auton();
                    sleep(500);
                    claw.open_claw_left();
                    sleep(1000);
                    arm.arm_fold();
                    sleep(500);
                })
                .back(4)
                .strafeLeft(35)
                .forward(10)
                .build();
    }

}

