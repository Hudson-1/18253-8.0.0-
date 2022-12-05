
package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.VisionPole;

@Disabled
@Config
@TeleOp
public class TestPole extends LinearOpMode {
    boolean toggle = false;
    boolean lastPress = false;
    public static double initialTestAngle = -45;
    public static double angleIncrease = 5;
    private double midline1;
    private double midline2;
    private double midline3;
    private double midline4;
    private double midline5;
    private double midline6;
    private double midline7;
    private double midline8;
    private double midline9;
    private double midline10;


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

        VisionPole visionpole = new VisionPole();
        visionpole.init(hardwareMap);

        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

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
                    drive.turn(initialTestAngle);
                    midline1 = visionpole.getDistanceFromPoleCenterToImageCenter();
                    sleep(1000);
                    drive.turn (angleIncrease);
                    midline2 = visionpole.getDistanceFromPoleCenterToImageCenter();
                    sleep(1000);
                    drive.turn (angleIncrease);
                    midline3 = visionpole.getDistanceFromPoleCenterToImageCenter();
                    sleep(1000);
                    drive.turn (angleIncrease);
                    midline4 = visionpole.getDistanceFromPoleCenterToImageCenter();
                    sleep(1000);
                    drive.turn (angleIncrease);
                    midline5 = visionpole.getDistanceFromPoleCenterToImageCenter();
                    sleep(1000);
                    drive.turn (angleIncrease);
                    midline6 = visionpole.getDistanceFromPoleCenterToImageCenter();
                    sleep(1000);
                    midline7 = visionpole.getDistanceFromPoleCenterToImageCenter();
                    drive.turn (angleIncrease);
                    midline8 = visionpole.getDistanceFromPoleCenterToImageCenter();
                    sleep(1000);
                    drive.turn (angleIncrease);
                    midline9 = visionpole.getDistanceFromPoleCenterToImageCenter();
                    sleep(1000);
                    drive.turn (angleIncrease);
                    midline10 = visionpole.getDistanceFromPoleCenterToImageCenter();




/*
drive.setPoseEstimate(startPose);



                    // WE CREATE THE APPROPRIATE TRAJECTORY TO GET TO THAT POINT
                    TrajectorySequence poleAim = drive.trajectorySequenceBuilder(startPose)
                            .turn(testAngle)
                            .build();

                    // WE DRIVE THAT TRAJECTORY
                    drive.followTrajectorySequence(poleAim); // note -- this is NOT async so we don't take a reading until it is done
*/

                    // THEN GET THE READINGS
 //                   double currentAngle = visionpole.getAngle();
 //                   double currentDistance = visionpole.getDistance();
 //                   double trajectoryX = (currentDistance * Math.sin(Math.toRadians(currentAngle)));
 //                   double trajectoryY = (currentDistance * Math.cos(Math.toRadians(currentAngle)));

                    telemetry.addData("Angle: ", initialTestAngle);
                    telemetry.addData("Midline: ", midline1);
                    telemetry.addData("Angle: ", initialTestAngle + angleIncrease);
                    telemetry.addData("Midline: ", midline2);
                    telemetry.addData("Angle: ", initialTestAngle + 3*(angleIncrease));
                    telemetry.addData("Midline: ", midline3);
                    telemetry.addData("Angle: ", initialTestAngle + 4*(angleIncrease));
                    telemetry.addData("Midline: ", midline4);
                    telemetry.addData("Angle: ", initialTestAngle + 5*(angleIncrease));
                    telemetry.addData("Midline: ", midline5);
                    telemetry.addData("Angle: ", initialTestAngle + 6*(angleIncrease));
                    telemetry.addData("Midline: ", midline6);
                    telemetry.addData("Angle: ", initialTestAngle + 7*(angleIncrease));
                    telemetry.addData("Midline: ", midline7);
                    telemetry.addData("Angle: ", initialTestAngle + 8*(angleIncrease));
                    telemetry.addData("Midline: ", midline8);
                    telemetry.addData("Angle: ", initialTestAngle + 9*(angleIncrease));
                    telemetry.addData("Midline: ", midline9);
                    telemetry.addData("Angle: ", initialTestAngle + 10*(angleIncrease));
                    telemetry.addData("Midline: ", midline10);
                    telemetry.update();

                    // WHEN DONE WE CEDE CONTROL BACK TO THE DRIVER
                    if (!drive.isBusy()) {
                        currentMode = states.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }
}

