package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(300), Math.toRadians(300), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, -64, Math.toRadians(270)))
                           //     .splineTo(new Vector2d(36, -40), Math.toRadians(90))
                                .setReversed(true)
                                .splineTo(new Vector2d(36, -36), Math.toRadians(90))
                                .splineTo(new Vector2d(29, -9), Math.toRadians(120))
                                .setReversed(false)
                                .splineTo(new Vector2d(55, -16), Math.toRadians(0))
                                .setVelConstraint(new MecanumVelocityConstraint(10, 12))
                                .forward(10)
                                .resetVelConstraint()
                                .setReversed(true)
                                .splineTo(new Vector2d(30, -7), Math.toRadians(120))
                            //    .UNSTABLE_addTemporalMarkerOffset(0.3, lift::grab)
                                .turn(Math.toRadians(-120))
                                .setReversed(true)
                                .lineTo(new Vector2d(50, -15))
                                .setVelConstraint(new MecanumVelocityConstraint(20, 12))
                                .back(7)
                                .resetVelConstraint()
                                // .splineTo(new Vector2d(60, -15), Math.toRadians(0))
                            //    .addTemporalMarker(lift::release)
                                .waitSeconds(.3)
                        //        .addTemporalMarker(() -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                                .waitSeconds(.1)
                                .setReversed(true)
                          //      .UNSTABLE_addTemporalMarkerOffset(0.2, lift::back)
                              //  .setReversed(false)
                              //  .forward(5)
                             //   .turn(Math.toRadians(-105))
                             //   .setReversed(true)
                             //   .back(20)

                              //  .setReversed(true)
                              //  .splineToLinearHeading(new Pose2d(55, -12, Math.toRadians(180)), Math.toRadians(0))
                               // .waitSeconds(1)
                             //   .lineToLinearHeading(new Pose2d(55, -12, Math.toRadians(180)))
                              //  .waitSeconds(1)
                             //   .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(135)))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}