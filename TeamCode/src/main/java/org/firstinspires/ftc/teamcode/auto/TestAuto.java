package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
@Autonomous
public class TestAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(gamepad1, gamepad2, hardwareMap,true);
        SampleMecanumDrive drive = robot.getDriveClass().getDrive();
        Lift lift = robot.lift;

        // The starting position of the robot on the field:
        Pose2d startPose = new Pose2d(0, -64, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        Vision vision = new Vision();
        vision.init(hardwareMap);

        Vision.Detection_States target = Vision.Detection_States.ONE;

        while (!isStopRequested() && !opModeIsActive()) {
            target = vision.returnVisionState();
            telemetry.addData("Vision condition is: ",target);
            telemetry.update();
        }

        Pose2d parkingOption3 = new Pose2d(64,-60,Math.toRadians(270)); // TODO edit this
        Pose2d parkingOption2 = new Pose2d(0,-59,Math.toRadians(270)); // TODO edit this
        Pose2d parkingOption1 = new Pose2d(-53,-60,Math.toRadians(270)); // TODO edit this

        Pose2d chosenTarget;  // use this later in your parking routine
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
                .lineTo(new Vector2d(0, -60))
                .lineToLinearHeading(chosenTarget)
                .back(50)
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
