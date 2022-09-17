package org.firstinspires.ftc.teamcode.subsystems.subclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShooterTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Shooter shooter = new Shooter(gamepad1,true);
        shooter.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            shooter.setRpm(1000);
            shooter.update();
        }
    }
}
