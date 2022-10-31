package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class MecDriveAuto extends MecDrive {
    SampleMecanumDrive drive;
    Gamepad g;

    public MecDriveAuto(Gamepad g) {
        super(g);
    }

    @Override
    public void init(HardwareMap map) {
        drive = new SampleMecanumDrive(map);
    }

    @Override
    public void update() {
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
