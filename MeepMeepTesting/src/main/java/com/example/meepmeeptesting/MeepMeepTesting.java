package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(300), Math.toRadians(300), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, -64, Math.toRadians(90)))
                                .lineTo(new Vector2d(36, -12))
                                .turn(Math.toRadians(45))
                                .waitSeconds(1) // repeat spot
                                .lineToLinearHeading(new Pose2d(55, -12, Math.toRadians(180)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(135)))
                                .waitSeconds(1) // repeat spot
                                .turn(Math.toRadians(45))
                                .lineTo(new Vector2d(55, -12))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(36, -12))
                                .turn(Math.toRadians(-45))
                                .waitSeconds(1) // repeat spot
                                .turn(Math.toRadians(45))
                                .lineTo(new Vector2d(55, -12))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(36, -12))
                                .turn(Math.toRadians(-45))
                                .waitSeconds(1) // repeat spot
                                .turn(Math.toRadians(45))
                                .lineTo(new Vector2d(55, -12))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(36, -12))
                                .turn(Math.toRadians(-45))
                                .waitSeconds(1) // repeat spot
                                .turn(Math.toRadians(45))
                                .lineTo(new Vector2d(55, -12))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(36, -12))
                                .turn(Math.toRadians(-45))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}