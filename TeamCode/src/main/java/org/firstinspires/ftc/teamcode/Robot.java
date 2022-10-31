package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    MecDrive drive;
    Intake intake;
 //   Shooter shooter;
    Lift lift;
    List<Subsystem> list;
    // If the boolean is true, run auto version of robot, if false run the tele version of robot
    boolean auto;


    public Robot(Gamepad gamepad1, Gamepad gamepad2, HardwareMap map, boolean auto) {
        this.auto = auto;
        drive = new MecDrive(gamepad1,auto);
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

    public MecDrive getDriveClass() {
        return drive;
    }

}
