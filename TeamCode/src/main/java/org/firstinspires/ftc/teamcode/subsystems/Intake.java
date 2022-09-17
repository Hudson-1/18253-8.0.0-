package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements Subsystem {
    DcMotor m1;
    DcMotor m2;
    boolean isTwoMotor;
    Gamepad g1;
    Gamepad g2;

    boolean toggle = false;
    boolean lastPress = false;

    public static double max = 1.0;
    public static double capMin = 1.0;
    public static double capMax = 1.0;

    public Intake(Gamepad g1, Gamepad g2, boolean isTwoMotor) {
        this.isTwoMotor = isTwoMotor;
        this.g1 = g1;
        this.g2 = g2;
    }

    @Override
    public void init(HardwareMap map) {
        m1 = map.get(DcMotor.class, "i1");
        if (isTwoMotor)
            m2 = map.get(DcMotor.class, "i2");

//        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void update() {
        triggerIntake();
    }

    public void toggleIntake() {
        boolean button = g1.a;

        if(button) {
            lastPress = true;
        }

        if(lastPress && !button) {
            toggle = !toggle;
            lastPress = false;
        }

        if(toggle) {
            setPower(capMax);
        } else {
            setPower(0);
        }

    }

    public void triggerIntake() {
        double power = g1.right_trigger - g1.left_trigger;
        power *= max;
        power = Range.clip(power, -capMin, capMax);

        setPower(power);
    }

    public void setPower(double power) {
        m1.setPower(power);
        if(isTwoMotor) m2.setPower(power);
    }

    @Override
    public Telemetry telemetry() {
        return null;
    }
}
