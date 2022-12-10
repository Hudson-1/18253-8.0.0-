package org.firstinspires.ftc.teamcode.tele;

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

@TeleOp
public class PoleAimTele extends LinearOpMode {
    boolean toggle = false;
    boolean lastPress = false;
    static double range = 2.0; //number must be positive

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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

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

                    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
                    drive.setPoseEstimate(startPose);

                    // Initialize the camera
                    VisionPole visionPole = new VisionPole();
                    visionPole.init(hardwareMap);

                    // Update the telemetry with the latest data
                    telemetry.addData("width of the closet pole: ", visionPole.getWidthOfTheClosestPole());
                    telemetry.addLine();
                    telemetry.addData("distance between pole and center: ", visionPole.getDistanceFromPoleCenterToImageCenter());
                    telemetry.addLine();
                    telemetry.update();

                    // Get the angle that we need to turn in order for our camera to face the pole straight
                    double angle = visionPole.getAngle();

                    while (!IsWithinRange(angle)) {
                        // If the angle is not zero, it means we need to turn
                        // Update the telemetry with the angle data
                        telemetry.addData("angle we need to turn: ", angle);
                        telemetry.addLine();
                        telemetry.update();

                        // Turn
                        drive.turnAsync(Math.toRadians(angle));

                        // Get the latest angle
                        angle = visionPole.getAngle();
                    }

                    // Get the distance between camera and pole
                    double distance = visionPole.getDistance();

                    // Update the telemetry with the distance data
                    telemetry.addData("distance we need to move forward: ", distance);
                    telemetry.addLine();
                    telemetry.update();

                    // Move
                    startPose = new Pose2d(0, 0, Math.toRadians(0));
                    drive.setPoseEstimate(startPose);
                    Trajectory Distance = drive.trajectoryBuilder(startPose)
                            .forward(distance)
                            .build();
                    drive.followTrajectoryAsync(Distance);

                    // WHEN DONE WE CEDE CONTROL BACK TO THE DRIVER
                    if (!drive.isBusy()) {
                        currentMode = states.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }
}