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
                    // IGNORE THIS PART. IT'LL BE DONE BY HELPER CLASS
                    // width = the width of the biggest pole found in our vision pipeline
                    // double width = visionpole.getMid();

                    // THIS DEFINES OUR CURRENT LOCATION AS 0,0 WITH A 0 HEADING
                    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
                    drive.setPoseEstimate(startPose);

                    // WE CREATE THE APPROPRIATE TRAJECTORY TO GET TO THAT POINT, PULLING VARIABLES FROM THE HELPER CLASS
                    Trajectory poleAim = drive.trajectoryBuilder(startPose)
                            .lineToLinearHeading(new Pose2d(4, 4, Math.toRadians(45)))
                            //2, 2, 20 are placekeepers.
                            .build();

                    // WE DRIVE THAT TRAJECTORY
                    drive.followTrajectoryAsync(poleAim);

                    // WHEN DONE WE CEDE CONTROL BACK TO THE DRIVER
                    if (!drive.isBusy()) {
                        currentMode = states.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }
}