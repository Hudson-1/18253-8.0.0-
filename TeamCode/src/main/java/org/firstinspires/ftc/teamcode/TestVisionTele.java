package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@TeleOp
public class TestVisionTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        Robot robot = new Robot(gamepad1, gamepad2, hardwareMap);
        Vision vision = new Vision();
        vision.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            robot.update();
            telemetry.addData("Vision Output:", vision.getColorNum());
            telemetry.update();
        }
    }
}
