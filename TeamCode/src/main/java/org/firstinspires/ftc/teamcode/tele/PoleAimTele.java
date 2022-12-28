package org.firstinspires.ftc.teamcode.tele;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.VisionPole;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class PoleAimTele extends LinearOpMode {
    boolean toggle = false;
    boolean lastPress = false;
    static double range = 3.0; //number must be positive

    // DEFINES THE TWO STATES -- DRIVER CONTROL OR AUTO ALIGNMENT
    public enum states {
        DRIVER_CONTROL,
        AUTO_ALIGN,
    }

    private states currentMode = states.DRIVER_CONTROL;

    private boolean IsWithinRange(double angle) {
        if ((angle > (0-range)) && (angle < range)) {
            return true;
        }
        return false;
    }

    @Override
    public void runOpMode() throws InterruptedException {

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

        VisionPole visionPole = new VisionPole();
        visionPole.init(hardwareMap);

        while (opModeIsActive()) {
            robot.update();
            dashboardTelemetry.update();

            switch (currentMode) {
                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    // PRESSING BUTTON TRIGGERS THE AUTO-ALIGN ROUTINE
                    boolean button = gamepad1.right_stick_button;

                    if (button) {
                        lastPress = true;
                    }

                    if (lastPress && !button) {
                        toggle = !toggle;
                        lastPress = false;
                    }

                    if (toggle) {
                        currentMode = states.AUTO_ALIGN;

                    } else {
                        currentMode = states.DRIVER_CONTROL;
                    }
                    break;

                case AUTO_ALIGN:

                 //   numberOfTimesItIsGettingCalled++;
                    /*
                    After calculating the angle we need to turn and the distance we need to drive:
                        - We start by only turning the angle.
                        - Then we have a loop:
                        Update the camera image and reanalyze it.
                            If the midline of the pole is not the same as the center of the screen (plus or minus a small amount) we repeat the loop.
                            If the midline is the same as the center if the screen (plus or minus), we move on.
                        - Then we drive forward the specified distance.
                       At this point we SHOULD be where we need to be.
                       But we could always run the loop one more time if we find it’s still not accurate enough.
                     */

                    // Update the telemetry with the latest data
                    telemetry.addData("width of the closet pole: ", visionPole.getWidthOfTheClosestPole());
                    telemetry.addData("distance between pole and center: ", visionPole.getDistanceFromPoleCenterToImageCenter());
                    telemetry.addData("the number of Contours: ", visionPole.getNumberOfContours());

                    // Get the angle that we need to turn in order for our camera to face the pole straight
                    double angle = 0 - visionPole.getAngle(); // we need to negate this value so the robot can understand

                    if (!IsWithinRange(angle)) {
                        if (angle > 0) {
                            telemetry.addData("turn left: ", angle);
                        } else if (angle < 0) {
                            telemetry.addData("turn right: ", -angle);
                        }
                        drive.turn(Math.toRadians(angle));
                    } else {
                        telemetry.addData("angle falls within the range: ", angle);

                        // Get the distance between camera and pole from the perceived focal length
                        double distance = visionPole.getDistanceFromFocalLength();

                        // Update the telemetry with the distance data
                        telemetry.addData("distance we need to move forward: ", distance);

                        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180)); // this is 180 bc bot is backwards
                        drive.setPoseEstimate(startPose);
                        TrajectorySequence Distance = drive.trajectorySequenceBuilder(startPose)
                                .forward(-distance) // note the negative to make it go forwards
                                .build();
                        drive.followTrajectorySequence(Distance);

                    }

                    telemetry.update();

                    lastPress = true;
                    currentMode = states.DRIVER_CONTROL;

                    break;
            }
        }
    }
}