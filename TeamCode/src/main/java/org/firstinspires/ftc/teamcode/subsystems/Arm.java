package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm implements Subsystem {
    private boolean isTwoMotor;
    DcMotorEx left;
    DcMotorEx right;
    DcMotorEx encoder;

    Gamepad gamepad1;
    Gamepad gamepad2;

    ElapsedTime timer;

    double previousError;
    double previousTime;

//    public static final double TICKS_PER_REV = 28.0 * 50.9;
//    public static final double GEAR_RATIO = 20.0 / 24.0;

    public static final double TICKS_PER_REV = 8192;
    public static final double GEAR_RATIO = 1.0;

    public static double target;

    public static PIDCoefficients coefficients = new PIDCoefficients(0, 0, 0);
    public static double kCos = 0;
    public static double kS = 0;
    private VoltageSensor batteryVoltageSensor;
    public static double range = 1.0;

    public Arm(Gamepad g1, Gamepad g2, boolean isTwo) {
        gamepad1 = g1;
        gamepad2 = g2;
        isTwoMotor = isTwo;
    }

    @Override
    public void init(HardwareMap map) {
        timer = new ElapsedTime();

        left = map.get(DcMotorEx.class, "la");
        right = map.get(DcMotorEx.class, "ra");
        encoder = map.get(DcMotorEx.class, "ri");

        right.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        batteryVoltageSensor = map.voltageSensor.iterator().next();

        resetPos();

        target = 0;
    }

    public void resetPos() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        double currentTime = System.currentTimeMillis() / 1000.0;
        double currentPosition = armAngle();
        double error = target - currentPosition;

        double p = coefficients.kP * error;
        double d = coefficients.kD * (error - previousError)/(currentTime - previousTime);
        double ks = kS * Math.signum(error);
        double kcos = Math.cos(-armAngle()) * kCos;

        double pid = p + d;

        pid *= -1.0;

        pid += (ks + kcos);
        pid  = Range.clip(pid, -0.7, 0.8);

        if(target == 0) {
            pid = Range.clip(pid, -0.35, 0.35);
        }

        if(Math.abs(error) < 0.01 || (target == 0 && Math.abs(currentPosition) < 0.2)) {
            pid = 0;
        }

        setPower(pid);

        previousError = error;
        previousTime = currentTime;
    }

    @Override
    public Telemetry telemetry() {
        return null;
    }

    public double armAngle() {
        return ticksToRad(encoder.getCurrentPosition());
    }

    public static double ticksToRad(double ticks) {
        return 2.0 * Math.PI * (ticks / (TICKS_PER_REV * GEAR_RATIO));
    }

    public void setPower(double power) {
        left.setPower(power);
        if(isTwoMotor) right.setPower(power);
    }
}