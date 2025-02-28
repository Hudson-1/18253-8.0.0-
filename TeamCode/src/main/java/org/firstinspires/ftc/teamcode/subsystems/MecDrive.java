package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class MecDrive implements Subsystem {
    SampleMecanumDrive drive;
    Gamepad g;
    boolean auto;

    public MecDrive(Gamepad g, boolean auto) {
        this.g = g;
        this.auto = auto;
    }

    @Override
    public void init(HardwareMap map) {
        drive = new SampleMecanumDrive(map);
    }

    @Override
    public void update() {
        if (!auto) {
            Pose2d drivePower = new Pose2d(-g.left_stick_y, -g.left_stick_x, -g.right_stick_x);
            drive.setWeightedDrivePower(drivePower);
        }
        drive.update();
    }

    @Override
    public Telemetry telemetry() {
        return null;
    }

    public SampleMecanumDrive getDrive() {
        return drive;
    }

}
