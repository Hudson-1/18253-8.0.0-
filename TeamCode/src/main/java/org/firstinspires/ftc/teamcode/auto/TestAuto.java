package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
@Autonomous
public class TestAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(gamepad1, gamepad2, hardwareMap, true,telemetry);
        SampleMecanumDrive drive = robot.getDriveClass().getDrive();

        // The starting position of the robot on the field:
        Pose2d startPose = new Pose2d(36, -64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);


        Lift lift = robot.getLift();

        // The trajectory that the robot follows during the auto
        TrajectorySequence Traj = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(36, -12))
                .turn(Math.toRadians(45))
                .addTemporalMarker(lift::slidesHigh)
                .waitSeconds(0.3)
                .addTemporalMarker(lift::front)
                .waitSeconds(0.3)
                .addTemporalMarker(lift::grab)
                .waitSeconds(0.3)
                .addTemporalMarker(lift::release)
                .waitSeconds(0.3)
                .addTemporalMarker(lift::stack)
                .addTemporalMarker(() -> lift.slideStack(7)) // rinse and repeat through 1, 5 is when there are 5 cones in the stack
                .waitSeconds(1)
                .addTemporalMarker(lift::grab)
                .lineToLinearHeading(new Pose2d(55, -12, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(135)))
                .waitSeconds(1) // repeat spot
                .build();

        robot.getLift().grab();
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
