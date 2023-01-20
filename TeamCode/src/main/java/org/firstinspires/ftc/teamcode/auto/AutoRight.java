package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auton.AprilTagAutonomousInitDetectionExample;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
@Autonomous
public class AutoRight extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(gamepad1, gamepad2, hardwareMap,true);
        SampleMecanumDrive drive = robot.getDriveClass().getDrive();
        Lift lift = robot.lift;
        // The starting position of the robot on the field:
        Pose2d startPose = new Pose2d(36, -64, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        Vision vision = new Vision();
        vision.init(hardwareMap);

        Vision.Detection_States target = Vision.Detection_States.ONE;

     //   AprilTagAutonomousInitDetectionExample aprilTag = new AprilTagAutonomousInitDetectionExample();
      //  AprilTagAutonomousInitDetectionExample.April_Tag_States new_target = AprilTagAutonomousInitDetectionExample.April_Tag_States.ONE;



        while (!isStopRequested() && !opModeIsActive()) {
            target = vision.returnVisionState();
            //new_target = aprilTag.visionLoop();
          //  telemetry.addData("Vision condition is: ",new_target);
            telemetry.addData("Vision condition is: ",target);
            telemetry.update();
        }

        double parkingOption3 = -30; // TODO edit this
        double parkingOption2 = -6; // TODO edit this
        double parkingOption1 = 22; // TODO edit this

        double chosenTarget;  // use this later in your parking routine
       // switch (new_target) {
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


     //   Lift lift = robot.getLift();

        // The trajectory that the robot follows during the auto
        TrajectorySequence Traj = drive.trajectorySequenceBuilder(startPose)
//test
        // ----DRIVE FORWARD, LINE UP WITH POLE, MOVE TO POLE----
                .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                .setReversed(true)
                .splineTo(new Vector2d(36, -36), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                .UNSTABLE_addTemporalMarkerOffset(-.9, lift::back)
                .splineTo(new Vector2d(30, -10), Math.toRadians(120))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0,lift::release)
                //deposit
               // .waitSeconds(.15)
               /* .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset( .55, lift::front5)
                .UNSTABLE_addTemporalMarkerOffset(.75, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                .splineTo(new Vector2d(55, -17), Math.toRadians(0))
                .setVelConstraint(new MecanumVelocityConstraint(20, 12))
                .forward(7)
                .resetVelConstraint()
                .addTemporalMarker(lift::grab)
                .waitSeconds(.2)
                .addTemporalMarker(() -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                .waitSeconds(.1)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.2, lift::back)
                .splineTo(new Vector2d(28, -7), Math.toRadians(130))
               // .splineTo(new Vector2d(30, -10), Math.toRadians(105))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0,lift::release)
                // end of +1
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0.55, lift::front4)
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                .splineTo(new Vector2d(55, -17), Math.toRadians(0))
                .setVelConstraint(new MecanumVelocityConstraint(20, 12))
                .forward(9)
                .resetVelConstraint()
                .addTemporalMarker(lift::grab)
                .waitSeconds(.2)
                .addTemporalMarker(() -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                .waitSeconds(.1)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.2, lift::back)
                .splineTo(new Vector2d(28, -7), Math.toRadians(130))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0,lift::release)
           //     .waitSeconds(.5)
                // end of +2
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0.55, lift::front3)
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                .splineTo(new Vector2d(55, -17), Math.toRadians(0))
                .setVelConstraint(new MecanumVelocityConstraint(20, 12))
                .forward(12)
                .resetVelConstraint()
                .addTemporalMarker(lift::grab)
                .waitSeconds(.2)
                .addTemporalMarker(() -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                .waitSeconds(.1)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.2, lift::back)
                .splineTo(new Vector2d(28, -7), Math.toRadians(130))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0,lift::release)*/
                //adding going down for park

               // .waitSeconds(.4)
                // end of +3
           /*     .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0.5, lift::front2)
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                .splineTo(new Vector2d(55, -17), Math.toRadians(0))
                .setVelConstraint(new MecanumVelocityConstraint(20, 12))
                .forward(12)
                .resetVelConstraint()
                .addTemporalMarker(lift::grab)
                .waitSeconds(.3)
                .addTemporalMarker(() -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                .waitSeconds(.1)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.2, lift::back)
                .splineTo(new Vector2d(30, -7), Math.toRadians(120))
                .waitSeconds(.2)
                .addTemporalMarker(lift::release)
                .waitSeconds(.4)
                // end of +4
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0.5, lift::front1)
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> lift.setLiftPosition(Lift.LiftState.REST, 0))
                .UNSTABLE_addTemporalMarkerOffset(0, lift::intakeout)
                .splineTo(new Vector2d(55, -17), Math.toRadians(0))
                .setVelConstraint(new MecanumVelocityConstraint(20, 12))
                .forward(13)
                .resetVelConstraint()
                .addTemporalMarker(lift::grab)
                .waitSeconds(.3)
                .addTemporalMarker(() -> lift.setLiftPosition(Lift.LiftState.HIGH, 0))
                .waitSeconds(.1)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.2, lift::back)
                .splineTo(new Vector2d(30, -7), Math.toRadians(120))
                .waitSeconds(.2)
                .addTemporalMarker(lift::release)
                .waitSeconds(.4)*/
                // end of +5
                .forward(9)
                .turn(Math.toRadians(-120))
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
