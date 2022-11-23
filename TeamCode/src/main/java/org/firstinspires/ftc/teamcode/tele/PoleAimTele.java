package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
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

    // DEFINES THE TWO STATES -- DRIVER CONTROL OR AUTO ALIGNMENT
    enum Mode {
        DRIVER_CONTROL,
        AUTO_ALIGN
    }

    private Mode currentMode = Mode.DRIVER_CONTROL;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        Robot robot = new Robot(gamepad1, gamepad2, hardwareMap, false);

        // TO BE FIGURED OUT -- WHEN SHOULD THE CAMERA TURN ON? WILL IT BE ON THE ENTIRE TIME?
        VisionPole visionpole = new VisionPole();
        visionpole.init(hardwareMap);

        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDrive
                drive = new SampleMecanumDrive(hardwareMap);

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

                    // PRESSING A BUTTON TRIGGERS THE AUTO-ALIGN ROUTINE

                    if (gamepad1.back) {    // Hudson, you may have another way of doing this

                        // THIS DEFINES OUR CURRENT LOCATION AS 0,0 WITH A 0 HEADING
                        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
                        drive.setPoseEstimate(startPose);

                        // WE DO THE MATH TO DETERMINE HOW FAR WE NEED TO TURN AND MOVE TO BE ALIGNED

                        // WE CREATE THE APPROPRIATE TRAJECTORY TO GET TO THAT POINT
                        Trajectory poleAim = drive.trajectoryBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(2, -1, Math.toRadians(10)))
                                // x, y, angle will be variables from the math
                                .build();

                        // WE DRIVE THAT TRAJECTORY
                        drive.followTrajectoryAsync(poleAim);

                        // WE SWITCH STATES WHILE WE DRIVE
                        currentMode = Mode.AUTO_ALIGN;
                    }
                    break;
                case AUTO_ALIGN:
                    // WHEN DONE WE CEDE CONTROL BACK TO THE DRIVER
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }
}