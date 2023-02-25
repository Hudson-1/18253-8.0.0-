package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

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
public class AprilAutonRight extends LinearOpMode
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
    int POSITION_1 = 11;
    int POSITION_2 = 12;
    int POSITION_3 = 13;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam2"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        FtcDashboard.getInstance().startCameraStream(camera, 0);  //added

        //HARDWARE MAPPING HERE etc.

        Robot robot = new Robot(gamepad1, gamepad2, hardwareMap,true);
        SampleMecanumDrive drive = robot.getDriveClass().getDrive();
        Lift lift = robot.lift;
        Intake intake = robot.intake;

        // The starting position of the robot on the field:
        Pose2d startPose = new Pose2d(36, -64, Math.toRadians(270));
        drive.setPoseEstimate(startPose);


        double parkingOption3 = -30; // TODO edit this
        double parkingOption2 = -3; // TODO edit this
        double parkingOption1 = 20; // TODO edit this

        double chosenTarget = parkingOption2;

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == POSITION_1 || tag.id == POSITION_2 || tag.id == POSITION_3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!");
                    tagToTelemetry(tagOfInterest);

                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

waitForStart();

        /* Actually do something useful */
        if(tagOfInterest.id == POSITION_1){
            chosenTarget = parkingOption1;
        }
        else if(tagOfInterest.id == POSITION_2){
            chosenTarget = parkingOption2;
        }
        else{
            chosenTarget = parkingOption3;
        }

        TrajectorySequence Traj = drive.trajectorySequenceBuilder(startPose)

                .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                .setReversed(true)
                .splineTo(new Vector2d(36, -36), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                .UNSTABLE_addTemporalMarkerOffset(-.9, lift::back)
                .splineTo(new Vector2d(28, -8), Math.toRadians(150))
                .addTemporalMarker(lift::release)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                .UNSTABLE_addTemporalMarkerOffset( .5, lift::front5)
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                .UNSTABLE_addTemporalMarkerOffset(1.6, lift::grab)
                .splineTo(new Vector2d(60, -16), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                .UNSTABLE_addTemporalMarkerOffset(.2, lift::back)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->intake.setPower(-.3))
                .setReversed(true)
                .splineTo(new Vector2d(29, -9), Math.toRadians(130))
                .addTemporalMarker(lift::release)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->intake.setPower(0))
                .setReversed(false)
                //cycle of dis
                .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                .UNSTABLE_addTemporalMarkerOffset( .5, lift::front4)
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                .UNSTABLE_addTemporalMarkerOffset(1.6, lift::grab)
                .splineTo(new Vector2d(62, -16), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                .UNSTABLE_addTemporalMarkerOffset(.2, lift::back)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->intake.setPower(-.3))
                .setReversed(true)
                .splineTo(new Vector2d(29, -9), Math.toRadians(130))
                .addTemporalMarker(lift::release)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->intake.setPower(0))
                .setReversed(false)
                //cycle of dis
                .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                .UNSTABLE_addTemporalMarkerOffset( .5, lift::front3)
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                .UNSTABLE_addTemporalMarkerOffset(1.6, lift::grab)
                .splineTo(new Vector2d(64, -16), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                .UNSTABLE_addTemporalMarkerOffset(.2, lift::back)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->intake.setPower(-.3))
                .setReversed(true)
                .splineTo(new Vector2d(29, -9), Math.toRadians(125))
                .addTemporalMarker(lift::release)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->intake.setPower(0))
                .setReversed(false)
                //cycle of dis
                .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                .UNSTABLE_addTemporalMarkerOffset( .5, lift::front2)
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                .UNSTABLE_addTemporalMarkerOffset(1.8, lift::grab)
                .splineTo(new Vector2d(66, -16), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                .UNSTABLE_addTemporalMarkerOffset(.2, lift::back)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->intake.setPower(-.3))
                .setReversed(true)
                .splineTo(new Vector2d(29, -9), Math.toRadians(117))
                .addTemporalMarker(lift::release)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->intake.setPower(0))
                .setReversed(false)
                .forward(9)
                .turn(Math.toRadians(-117))
                .UNSTABLE_addTemporalMarkerOffset(0, lift::intakein)
                .UNSTABLE_addTemporalMarkerOffset(0.55, lift::front1)
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                .forward(chosenTarget)

                .build();

        drive.followTrajectorySequenceAsync(Traj);
        while (opModeIsActive() && !isStopRequested()) {
            robot.update();
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nParking Location: %d", ((detection.id)-10)));
    }
}

