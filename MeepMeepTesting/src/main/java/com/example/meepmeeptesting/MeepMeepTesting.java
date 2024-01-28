package com.example.meepmeeptesting;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        /*
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .build()
                );

         */
        Pose2d startPose = new Pose2d(-40, 56, 270);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(15,14)
                .setConstraints(60, 40, Math.toRadians(180), Math.toRadians(180), 10.82)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5, 64, Math.toRadians(270)))
                                .forward(51)
                                .turn(Math.toRadians(180))
                                .waitSeconds(2)
                                .back(2)
                                .turn(Math.toRadians(90))
                                .waitSeconds(1)
                                .forward(18)
                                .waitSeconds(1)
                                .turn(Math.toRadians(180))
                                .forward(92)
                                .strafeLeft(25)
                                .waitSeconds(2)
                                .forward(12)
                                .back(12)
                                .strafeRight(23)
                                .forward(25)



                                //.strafeLeft(23)
                                //.forward(14)
                                /*
                                .back(5)
                                .strafeRight(23)
                                .turn(Math.toRadians(180))
                                .forward(92)
                                .strafeLeft(22)
                                .forward(10)
                                .back(10)
                                .strafeRight(22)
                                .forward(10)
                                .back(102)
                                .turn(Math.toRadians(180))
                                .strafeLeft(23)
                                .forward(7)

                                 */

                                /*
                                .turn(Math.toRadians(50))
                                .forward(4)
                                .back(4)
                                .turn(Math.toRadians(-50))
                                .waitSeconds(1)
                                .back(19)
                                .waitSeconds(1)
                                .turn(Math.toRadians(90))
                                .waitSeconds(2)
                                .forward(71)
                                .strafeRight(14.25)
                                .forward(18.25)
                                .back(4)
                                .strafeLeft(17)
                                .forward(9)
                                */

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}