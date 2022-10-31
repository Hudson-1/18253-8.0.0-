package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecDriveAuto;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class TestAuto extends LinearOpMode {

    Robot robot = new Robot(gamepad1, gamepad2, hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.setDrive(new MecDriveAuto(gamepad1), hardwareMap);
        SampleMecanumDrive drive = robot.getMecDriveAuto().getDrive();
        // The type of Roadrunner drive we are using:

        // The starting position of the robot on the field:
        Pose2d startPose = new Pose2d(-35, 64, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        // The trajectory that the robot follows during the auto
        Trajectory testTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(0, 64), Math.toRadians(0))
                .build();

        while (!isStarted()) {
            telemetry.addData("Telemetry Test", 0);
            telemetry.update();
        }

        drive.followTrajectoryAsync(testTrajectory);
        while (opModeIsActive() && !isStopRequested()) {
            robot.update();
        }
    }
}
