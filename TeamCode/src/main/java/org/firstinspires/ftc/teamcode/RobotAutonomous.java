package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "SciRavens-Autonomous")
public class RobotAutonomous extends LinearOpMode {
    public Robot robot;
    public Slider slider;
    public Arm arm;
    public Claw claw;

    public AprilTag tag;
    public TgeDetection tge;
    String curAlliance = "red";
    public int zone = 2;

    private Trajectory traj2_1, traj2_2, traj2_3, traj2_4, traj2_5;
    TrajectorySequence trajseq;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        slider = new Slider(robot, gamepad2);
        arm = new Arm(robot, gamepad2);
        claw = new Claw(robot, gamepad2);
        tag = new AprilTag(robot);
        tge = new TgeDetection(robot.webcam);

        Pose2d startPose = new Pose2d(0, 0, 0);
        robot.sampleDrive.setPoseEstimate(startPose);
        traj2_1 = robot.sampleDrive.trajectoryBuilder(startPose)
                .forward(27)
                .build();
        traj2_2 = robot.sampleDrive.trajectoryBuilder(traj2_1.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .forward(40)
                .build();
        traj2_3 = robot.sampleDrive.trajectoryBuilder(traj2_2.end())
                .back(4)
                .build();
        traj2_4 = robot.sampleDrive.trajectoryBuilder(traj2_3.end())
                .strafeLeft(25)
                .build();
        traj2_5 = robot.sampleDrive.trajectoryBuilder(traj2_4.end())
                .forward(10)
                .build();

        startPose = new Pose2d(0, 0, 0);
        robot.sampleDrive.setPoseEstimate(startPose);
        trajseq = robot.sampleDrive.trajectorySequenceBuilder(startPose)
                .forward(27)
                .waitSeconds(2)
                .addTemporalMarker(() -> { claw.open_claw_right(); sleep(500); arm.arm_backdrop(); sleep(500);} )
                .waitSeconds(2)
                //.setTurnConstraint(5.0, 5.0)
                .turn(Math.toRadians(90))
                .waitSeconds(2)
                //.setAccelConstraint(robot.sampleDrive.getAccelerationConstraint(30.0))
                .forward(40)
                .waitSeconds(2)
                .addTemporalMarker(() -> { slider.auton(); sleep(500); claw.open_claw_left(); sleep(1000); arm.arm_fold();sleep(500);})
                .waitSeconds(2)
                .back(4)
                .strafeLeft(25)
                .forward(10)
                .build();

        waitForStart();
        if(opModeIsActive()) {
                //zone = tge.elementDetection(telemetry);
                zone = 2;
            }
            switch(zone) {
                case 1:
                    auton1();
                    break;
                case 2:
                    auton2();
                    break;
                case 3:
                    auton3();
                    break;
            }

    }

    public void auton1() {

    }
    public void auton2() {
        claw.close_claw_right();
        claw.close_claw_left();
        robot.sampleDrive.followTrajectorySequence(trajseq);;
        sleep(10000);
        /*
        robot.sampleDrive.followTrajectory(traj2_1);
        claw.open_claw_right();
        sleep(500);
        arm.arm_backdrop();
        sleep(500);
        robot.sampleDrive.turn(Math.toRadians(90));
        sleep(1000);
        robot.sampleDrive.followTrajectory(traj2_2);
        slider.auton();
        sleep(1000);
        arm.arm_backdrop();
        sleep(500);
        claw.open_claw_left();
        sleep(500);
        arm.arm_fold();
        sleep(500);
        robot.sampleDrive.followTrajectory(traj2_3);
        sleep(500);
        robot.sampleDrive.followTrajectory(traj2_4);
        sleep(1000);
        robot.sampleDrive.followTrajectory(traj2_5);
        sleep(1000);
        */
    }
    public void auton3() {

    }
}

