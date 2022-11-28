package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.VisionPole;

@TeleOp
public class PoleAimTele extends LinearOpMode {
    boolean toggle = false;
    boolean lastPress = false;

    // DEFINES THE TWO STATES -- DRIVER CONTROL OR AUTO ALIGNMENT
    public enum states {
        DRIVER_CONTROL,
        AUTO_ALIGN,
    }

    private states currentMode = states.DRIVER_CONTROL;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        Robot robot = new Robot(gamepad1, gamepad2, hardwareMap, false);

        // TO BE FIGURED OUT -- WHEN SHOULD THE CAMERA TURN ON? WILL IT BE ON THE ENTIRE TIME?
        VisionPole visionpole = new VisionPole();
        visionpole.init(hardwareMap);

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

                    /*THE PLAN

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


                    /*SAMPLE CODE

                    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
                    drive.setPoseEstimate(startPose);
                    TrajectorySequence Angle = drive.trajectoryBuilder(startPose)
                            .turn(Math.toRadians(ENTER ANGLE HERE))
                            .build();
                    drive.followTrajectoryAsync(Angle);

                    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
                    drive.setPoseEstimate(startPose);
                    TrajectorySequence Distance = drive.trajectoryBuilder(startPose)
                            .forward(INPUT DISTANCE)
                            .build();
                    drive.followTrajectoryAsync(Distance);

                    */




                    // OLD CODE

                    // THIS DEFINES OUR CURRENT LOCATION AS 0,0 WITH A 0 HEADING
                    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
                    drive.setPoseEstimate(startPose);

               //     double currentAngle = visionpole.getAngle();
               //     double currentDistance = visionpole.getDistance();
               //     double trajectoryX = (currentDistance * Math.sin(Math.toRadians(currentAngle)));
               //     double trajectoryY = (currentDistance * Math.cos(Math.toRadians(currentAngle)));

                    // WE CREATE THE APPROPRIATE TRAJECTORY TO GET TO THAT POINT
               //     Trajectory poleAim = drive.trajectoryBuilder(startPose)
               //             .lineToLinearHeading(new Pose2d(trajectoryX, trajectoryY, Math.toRadians(currentAngle)))
               //             .build();

                    telemetry.addData("Width: ", visionpole.getWidth()); // width of rectangle reported by vision
                    telemetry.addData("Midline: ", visionpole.getMid()); // distance from midline reported by vision
                    telemetry.addLine();
                 //   telemetry.addData("Trajectory X: ", trajectoryX); // the X we are feeding the trajectory
                 //   telemetry.addData("Trajectory Y: ", trajectoryY); // the y we are feeding the trajectory
                 //   telemetry.addData("Trajectory Angle: ", currentAngle); // the angle we are feeding the trajectory
                    telemetry.update();


                    // WE DRIVE THAT TRAJECTORY
                 //   drive.followTrajectoryAsync(poleAim);

                    // WHEN DONE WE CEDE CONTROL BACK TO THE DRIVER
                    if (!drive.isBusy()) {
                        currentMode = states.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }
}