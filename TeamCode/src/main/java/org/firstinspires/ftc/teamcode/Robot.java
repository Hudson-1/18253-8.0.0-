package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    MecDrive drive;
    Intake intake;
 //   Shooter shooter;
    Lift lift;
    List<Subsystem> list;


    public Robot(Gamepad gamepad1, Gamepad gamepad2, HardwareMap map) {
        drive = new MecDrive(gamepad1);
        intake = new Intake(gamepad1, gamepad2, true);

      //  shooter = new Shooter(gamepad1, true);
        lift = new Lift(gamepad1, true);
        list=new ArrayList<>();

        list.add(drive);
        list.add(intake);
     //   list.add(shooter);

        list.add(lift);

        for(Subsystem s : list) {
            s.init(map);
        }
    }

    public void update() {
        for(Subsystem s : list) {
            s.update();
        }
    }
}
