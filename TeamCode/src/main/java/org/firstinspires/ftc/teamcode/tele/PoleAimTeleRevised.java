package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.VisionPole;
import org.firstinspires.ftc.teamcode.subsystems.VisionPoleRevised;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import java.lang.Math;

@Config
@TeleOp
public class PoleAimTeleRevised extends LinearOpMode {
    boolean toggle = false;
    boolean lastPress = false;
    public static double angleRange = 6.0;     // smallest angle range that we can turn
    public static double distanceRange = 4.0;  // smallest distance range that we can move
    public static double distanceDelta = 1.0;
    static int timer = 500;             // milliseconds that we sleep for

    // DEFINES THE TWO STATES -- DRIVER CONTROL OR AUTO ALIGNMENT
    public enum states {
        DRIVER_CONTROL,
        AUTO_ALIGN,
    }

    private states currentMode = states.DRIVER_CONTROL;

    private boolean IsWithinAngleRange(double angle) {
        return (Math.abs(angle) < angleRange);
        }

    private boolean IsWithinDistanceRange(double distance) {
        return (Math.abs(distance) < distanceRange);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        Robot robot = new Robot(gamepad1, gamepad2, hardwareMap, false);

        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDrive drive = robot.getDriveClass().getDrive();

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        VisionPoleRevised visionPole = new VisionPoleRevised();
        visionPole.init(hardwareMap);

        while (opModeIsActive()) {

            try {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
            } catch (RobotCoreException e){

            }

            robot.update();
            dashboardTelemetry.update();

            if (currentGamepad1.right_stick_button && !previousGamepad1.right_stick_button) {
                currentMode = (currentMode == states.DRIVER_CONTROL) ? states.AUTO_ALIGN : states.DRIVER_CONTROL;
            }

            if (currentMode == states.DRIVER_CONTROL) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
            } else {
                boolean actions = false;

                // Get the angle that we need to turn
                double angle = 0 - visionPole.getAngle(); // we need to negate this value so the robot can understand

                if (!IsWithinAngleRange(angle)) {
                    if (angle > 0) {
                        telemetry.addData("turn left: ", angle);
                    } else if (angle < 0) {
                        telemetry.addData("turn right: ", -angle);
                    }
                    drive.turn(Math.toRadians(angle));
                    actions = true;

                } else {
                    telemetry.addData("angle falls within range: ", angle);

                    // Get the distance between camera and pole using perceived focal length
                    double distance = visionPole.getDistanceFromFocalLength();

                    if (!IsWithinDistanceRange(distance)) {

                        double delta = Math.abs(distance) - distanceRange;

                        if (delta < distanceDelta) {
                            delta = distanceDelta;
                        }

                        if (distance > 0) {
                            distance = delta;
                        } else {
                            distance = 0 - delta;
                        }

                        // Update the telemetry with the distance data
                        telemetry.addData("distance we need to move forward: ", distance);

                        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180)); // this is 180 bc bot is backwards
                        drive.setPoseEstimate(startPose);

                        TrajectorySequence Distance = drive.trajectorySequenceBuilder(startPose)
                                .forward(-distance) // note the negative to make it go forwards
                                .build();
                        drive.followTrajectorySequence(Distance);

                        actions = true;
                    }
                }

                telemetry.update();

                if (!actions) {     // If there are no actions being taken, we consider the job is done. Give back to the driver control mode.
                    gamepad1.rumbleBlips(3);
                    currentMode = states.DRIVER_CONTROL;
                } else {            // Otherwise, sleep to give other threads a chance to run
                    sleep(timer);
                }

            }

            idle();

        }
    }
}