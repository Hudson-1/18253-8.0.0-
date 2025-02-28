package org.firstinspires.ftc.teamcode.auto;

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

@Disabled
@Config
@Autonomous
public class AutoLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(gamepad1, gamepad2, hardwareMap,true);
        SampleMecanumDrive drive = robot.getDriveClass().getDrive();
        Lift lift = robot.lift;
        Intake intake = robot.intake;

        // The starting position of the robot on the field:
        Pose2d startPose = new Pose2d(-36, -64, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        Vision vision = new Vision();
        vision.init(hardwareMap);

        Vision.Detection_States target = Vision.Detection_States.ONE;

        //AprilTag aprilTag = new AprilTag();
     //   AprilTag.April_Tag_States new_target = AprilTag.April_Tag_States.ONE;


        while (!isStopRequested() && !opModeIsActive()) {
            target = vision.returnVisionState();
            //aprilTag.visionLoop();
           // new_target = aprilTag.visionLoop();
            telemetry.addData("Vision condition is: ",target);
            //telemetry.addData("Vision condition is: ",aprilTag.getSignalPos());
            telemetry.update();
        }

        double parkingOption3 = -24; // TODO edit this
        double parkingOption2 = .5; // TODO edit this
        double parkingOption1 = 24; // TODO edit this

        double chosenTarget = parkingOption2;  // use this later in your parking routine
        switch (target) {
            case ONE:
                chosenTarget = parkingOption1;
                break;
            case TWO:
                chosenTarget = parkingOption2;
                break;
            default:
                chosenTarget = parkingOption3;
                break;
        }


/*
        switch(aprilTag.getSignalPos()){
            case 1:
                chosenTarget = parkingOption1;
                break;
            case 2:
                chosenTarget = parkingOption2;
                break;
            case 3:
                chosenTarget = parkingOption3;
                break;
        }

 */


     //   Lift lift = robot.getLift();

        // The trajectory that the robot follows during the auto
        TrajectorySequence Traj = drive.trajectorySequenceBuilder(startPose)
//test
        // ----DRIVE FORWARD, LINE UP WITH POLE, MOVE TO POLE----
                .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                .setReversed(true)
                .splineTo(new Vector2d(-36, -36), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                .UNSTABLE_addTemporalMarkerOffset(-.9, lift::back)
                .splineTo(new Vector2d(-28, -8), Math.toRadians(30))
                .addTemporalMarker(lift::release)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                .UNSTABLE_addTemporalMarkerOffset( .5, lift::front5)
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                .UNSTABLE_addTemporalMarkerOffset(1.6, lift::grab)
                .splineTo(new Vector2d(-60, -16), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                .UNSTABLE_addTemporalMarkerOffset(.2, lift::back)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->intake.setPower(-.2))
                .setReversed(true)
                .splineTo(new Vector2d(-29, -9), Math.toRadians(50))
                .addTemporalMarker(lift::release)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->intake.setPower(0))
                .setReversed(false)
                //cycle of dis
                .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                .UNSTABLE_addTemporalMarkerOffset( .5, lift::front4)
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                .UNSTABLE_addTemporalMarkerOffset(1.6, lift::grab)
                .splineTo(new Vector2d(-62, -16), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                .UNSTABLE_addTemporalMarkerOffset(.2, lift::back)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->intake.setPower(-.2))
                .setReversed(true)
                .splineTo(new Vector2d(-29, -9), Math.toRadians(50))
                .addTemporalMarker(lift::release)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->intake.setPower(0))
                .setReversed(false)
                //cycle of dis
                .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                .UNSTABLE_addTemporalMarkerOffset( .5, lift::front3)
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                .UNSTABLE_addTemporalMarkerOffset(1.6, lift::grab)
                .splineTo(new Vector2d(-64, -16), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                .UNSTABLE_addTemporalMarkerOffset(.2, lift::back)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->intake.setPower(-.2))
                .setReversed(true)
                .splineTo(new Vector2d(-29, -9), Math.toRadians(55))
                .addTemporalMarker(lift::release)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->intake.setPower(0))
                .setReversed(false)
                //cycle of dis
                .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                .UNSTABLE_addTemporalMarkerOffset( .5, lift::front2)
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                .UNSTABLE_addTemporalMarkerOffset(1.8, lift::grab)
                .splineTo(new Vector2d(-66, -16), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                .UNSTABLE_addTemporalMarkerOffset(.2, lift::back)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->intake.setPower(-.2))
                .setReversed(true)
                .splineTo(new Vector2d(-29, -9), Math.toRadians(63))
                .addTemporalMarker(lift::release)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->intake.setPower(0))
                .setReversed(false)
                .forward(9)
                .turn(Math.toRadians(-63))
                .UNSTABLE_addTemporalMarkerOffset(0, lift::intakein)
                .UNSTABLE_addTemporalMarkerOffset(0.55, lift::front1)
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                .forward(chosenTarget)


//end

                //bring arm

                //end deposit
               // .lineTo(new Vector2d(0, -50))
              //  .lineToLinearHeading(chosenTarget)

                //.back(50)
                .build();


      //  robot.getLift().grab();
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
