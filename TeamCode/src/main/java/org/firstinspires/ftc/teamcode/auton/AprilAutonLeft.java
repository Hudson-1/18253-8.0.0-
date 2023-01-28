package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

//public class AprilAutonLeft {
    /*
     * Copyright (c) 2021 OpenFTC Team
     *
     * Permission is hereby granted, free of charge, to any person obtaining a copy
     * of this software and associated documentation files (the "Software"), to deal
     * in the Software without restriction, including without limitation the rights
     * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
     * copies of the Software, and to permit persons to whom the Software is
     * furnished to do so, subject to the following conditions:
     *
     * The above copyright notice and this permission notice shall be included in all
     * copies or substantial portions of the Software.
     * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
     * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
     * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
     * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
     * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
     * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
     * SOFTWARE.
     */
    @Config
    @Autonomous
    public class AprilAutonLeft extends LinearOpMode

    {
        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;

        static final double FEET_PER_METER = 3.28084;
        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        // UNITS ARE METERS
        double tagsize = 0.166;

        // Tag ID 11,12,13  from the 36h11 family
        int LEFT = 11;
        int MIDDLE = 12;
        int RIGHT = 13;

        AprilTagDetection tagOfInterest = null;

        @Override
        public void runOpMode()
        {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

            camera.setPipeline(aprilTagDetectionPipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_LEFT);
                }

                @Override
                public void onError(int errorCode) {}
            });
            telemetry.setMsTransmissionInterval(50);
            FtcDashboard.getInstance().startCameraStream(camera, 0);  //added
            /*
             * The INIT-loop:
             * This REPLACES waitForStart!
             */
            while (!isStarted() && !isStopRequested()) {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
                if(currentDetections.size() != 0) {
                    for(AprilTagDetection tag : currentDetections) {
                        if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                            tagOfInterest = tag;
                            break;
                        }
                    }
                }
                telemetry.update();
                sleep(20);
            }
            /*
             * The START command just came in: now work off the latest snapshot acquired
             * during the init loop.
             */
            /* Update the telemetry */

            Robot robot = new Robot(gamepad1, gamepad2, hardwareMap,true);
            SampleMecanumDrive drive = robot.getDriveClass().getDrive();
            Lift lift = robot.lift;
            Intake intake = robot.intake;

            // The starting position of the robot on the field:
            Pose2d startPose = new Pose2d(-36, -64, Math.toRadians(270));
            drive.setPoseEstimate(startPose);

            double parkingOption3 = -24; // TODO edit this
            double parkingOption2 = .5; // TODO edit this
            double parkingOption1 = 24; // TODO edit this

            double chosenTarget = parkingOption2;

            /* Actually do something useful */
            if(tagOfInterest.id == LEFT){
                chosenTarget = parkingOption1;
            }
            else if(tagOfInterest.id == MIDDLE){
                chosenTarget = parkingOption2;
            }
            else{
                chosenTarget = parkingOption3;
            }

            TrajectorySequence Traj = drive.trajectorySequenceBuilder(startPose)
//test
                    // ----DRIVE FORWARD, LINE UP WITH POLE, MOVE TO POLE----
                    .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                    .setReversed(true)
                    .splineTo(new Vector2d(-36, -36), Math.toRadians(90))
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                    .UNSTABLE_addTemporalMarkerOffset(-.9, lift::back)
                    .splineTo(new Vector2d(-28, -8), Math.toRadians(20))
                    .addTemporalMarker(lift::release)
                    .setReversed(false)
                    .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                    .UNSTABLE_addTemporalMarkerOffset( .5, lift::front5)
                    .UNSTABLE_addTemporalMarkerOffset(.6, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                    .UNSTABLE_addTemporalMarkerOffset(1.6, lift::grab)
                    .UNSTABLE_addTemporalMarkerOffset(1.8, () -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                    .UNSTABLE_addTemporalMarkerOffset(2.0, lift::back)
                    .splineTo(new Vector2d(-60, -14), Math.toRadians(180))
                    .setReversed(true)
                    //   .UNSTABLE_addTemporalMarkerOffset(0.2, lift::back)
                    .splineTo(new Vector2d(-29, -9), Math.toRadians(45))
                    .addTemporalMarker(lift::release)
                    .setReversed(false)
                    .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                    .UNSTABLE_addTemporalMarkerOffset( .5, lift::front4)
                    .UNSTABLE_addTemporalMarkerOffset(.6, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                    .UNSTABLE_addTemporalMarkerOffset(1.6, lift::grab)
                    .UNSTABLE_addTemporalMarkerOffset(1.8, () -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                    .UNSTABLE_addTemporalMarkerOffset(2.0, lift::back)
                    .splineTo(new Vector2d(-62, -14), Math.toRadians(180))
                    .setReversed(true)
                    .UNSTABLE_addTemporalMarkerOffset(0.2, lift::back)
                    .splineTo(new Vector2d(-29, -9), Math.toRadians(45))
                    .addTemporalMarker(lift::release)
                    .setReversed(false)
                    .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                    .UNSTABLE_addTemporalMarkerOffset( .5, lift::front3)
                    .UNSTABLE_addTemporalMarkerOffset(.6, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                    .UNSTABLE_addTemporalMarkerOffset(1.6, lift::grab)
                    .UNSTABLE_addTemporalMarkerOffset(1.8, () -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                    .UNSTABLE_addTemporalMarkerOffset(2.0, lift::back)
                    .splineTo(new Vector2d(-64, -14), Math.toRadians(180))
                    .setReversed(true)
                    .UNSTABLE_addTemporalMarkerOffset(0.2, lift::back)
                    .splineTo(new Vector2d(-29, -9), Math.toRadians(55))
                    .addTemporalMarker(lift::release)
                    .setReversed(false)
                    .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                    .UNSTABLE_addTemporalMarkerOffset( .5, lift::front2)
                    .UNSTABLE_addTemporalMarkerOffset(.6, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                    .UNSTABLE_addTemporalMarkerOffset(1.6, lift::grab)
                    .UNSTABLE_addTemporalMarkerOffset(1.8, () -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                    .UNSTABLE_addTemporalMarkerOffset(2.0, lift::back)
                    .splineTo(new Vector2d(-66, -14), Math.toRadians(180))
                    .setReversed(true)
                    .UNSTABLE_addTemporalMarkerOffset(0.2, lift::back)
                    .splineTo(new Vector2d(-29, -9), Math.toRadians(55))
                    .addTemporalMarker(lift::release)
                    .setReversed(false)
                    .forward(9)
                    .turn(Math.toRadians(-35))
                    .UNSTABLE_addTemporalMarkerOffset(0, lift::intakein)
                    .UNSTABLE_addTemporalMarkerOffset(0.55, lift::front1)
                    .UNSTABLE_addTemporalMarkerOffset(0.75, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                    .forward(chosenTarget)
                    // .addTemporalMarker(intake::toggleIntake)
                    // .addTemporalMarker(intake::)
                    .UNSTABLE_addTemporalMarkerOffset(0.2, intake::triggerIntake)
                    //bring arm
                    //end deposit
                    // .lineTo(new Vector2d(0, -50))
                    //  .lineToLinearHeading(chosenTarget)
                    //.back(50)
                    .build();

            while (!isStarted()) {
                telemetry.addData("Telemetry Test", 0);
                telemetry.update();
            }

            drive.followTrajectorySequenceAsync(Traj);
            while (opModeIsActive() && !isStopRequested()) {
                robot.update();
            }
        }
    }

