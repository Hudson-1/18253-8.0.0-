package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift implements Subsystem {
    private boolean isTwoMotor;
    DcMotorEx m1;
    DcMotorEx m2;
    Gamepad g;

    public static double SPOOL_SIZE_IN = 2.0; // radius in inches
    public static double MOTOR_RATIO = 27.4;
    public static double TICKS_PER_REV = MOTOR_RATIO*28.0;
    public static double GEAR_RATIO = 1;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;
    private double target;


    public Lift(Gamepad g, boolean isTwoMotor) {
        this.g = g;
        this.isTwoMotor = isTwoMotor;
        target = 0;
    }

    @Override
    public void init(HardwareMap map) {
        m1 = map.get(DcMotorEx.class, "l1");
        if(isTwoMotor)
            m2 = map.get(DcMotorEx.class, "l2");

        //        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        //        m2.setDirection(DcMotorSimple.Direction.REVERSE);

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void update() {
        double error = target - encoderTicksToInches(m1.getCurrentPosition());
        double pid = error*kP + Math.copySign(kF, error);

        if(target == 0 && encoderTicksToInches(m1.getCurrentPosition()) < 0.2) {
            pid = 0;
        }

        m1.setPower(pid);

        if(isTwoMotor) {
            m2.setPower(pid);
        }
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public static double encoderTicksToInches(double ticks) {
        return SPOOL_SIZE_IN * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * SPOOL_SIZE_IN / 60.0;
    }

    @Override
    public Telemetry telemetry() {
        return null;
    }
}
