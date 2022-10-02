package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lift implements Subsystem {
    private boolean isTwoMotor;
    DcMotorEx m1;
    DcMotorEx m2;
    Gamepad g;

    public static double SPOOL_SIZE_IN = 0.5; // radius in inches
    public static double MOTOR_RATIO = 3.7;
    public static double TICKS_PER_REV = MOTOR_RATIO * 28.0;
    public static double GEAR_RATIO = 1;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;
    private double target;

    // change claw

    public static double clawOpen = 0;
    public static double clawClose = 1;

    // change v4b

    public static double rest = 0.4;
    public static double front = 0;
    public static double back = 1;

    Servo v4bL;
    Servo v4bR;
    Servo claw;

    public enum States {
        REST,
        LOW,
        MID,
        HIGH,
        STACK,
        STACK_DEPOSIT
    }

    public enum LiftState {
        REST,
        LOW,
        MID,
        HIGH,
        STACK,
        CHECK
    }

    ElapsedTime timer;

    States state = States.REST;

    public Lift(Gamepad g, boolean isTwoMotor) {
        this.g = g;
        this.isTwoMotor = isTwoMotor;
        target = 0;
        timer = new ElapsedTime();
    }

    @Override
    public void init(HardwareMap map) {
        m1 = map.get(DcMotorEx.class, "ll");
        m2 = map.get(DcMotorEx.class, "lr");

        v4bL = map.get(Servo.class, "v4bl");
        v4bR = map.get(Servo.class, "v4br");
        claw = map.get(Servo.class, "claw");

        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m2.setDirection(DcMotorSimple.Direction.REVERSE);

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        resetServos();
    }

    public void resetServos() {
        v4bL.setPosition(front);
        v4bR.setPosition(1-front);
        claw.setPosition(clawOpen);
    }

    public void reset() {
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void update() {
        updatePID();

        switch(state) {
            case REST:
                resetServos();
                if(timer.milliseconds() < 600) {
                    setLiftPosition(LiftState.CHECK, 0);
                } else {
                    setLiftPosition(LiftState.REST, 0);
                }

                if(g.a) {
                    state = States.LOW;
                }

                if(g.b) {
                    state = States.MID;
                }

                if(g.right_bumper) {
                    state = States.HIGH;
                }

                break;
            case LOW:
                setLiftPosition(LiftState.LOW, 0);
                grab();

                if(getCurrentPosition() > 4.0) {
                    back();
                }

                if(g.left_bumper) {
                    timer.reset();
                    state = States.REST;
                }
                break;
            case MID:
                setLiftPosition(LiftState.MID, 0);
                grab();

                if(getCurrentPosition() > 4.0) {
                    back();
                }

                if(g.left_bumper) {
                    timer.reset();
                    state = States.REST;
                }
                break;
            case HIGH:
                setLiftPosition(LiftState.HIGH, 0);
                grab();

                if(getCurrentPosition() > 4.0) {
                    back();
                }

                if(g.left_bumper) {
                    timer.reset();
                    state = States.REST;
                }
                break;
            case STACK:

                break;
            case STACK_DEPOSIT:

                break;
        }
    }

    public void grab() {
        claw.setPosition(clawClose);
    }

    public void back() {
        v4bL.setPosition(back);
        v4bL.setPosition(1-back);

    }

    public void front() {
        v4bL.setPosition(front);
        v4bL.setPosition(1-front);
    }

    public void setLiftPosition(LiftState ls, double height) {
        switch(ls) {
            case REST:
                target = 0.0;
                break;
            case CHECK:
                target = 4.0; // this is your minimum position where your v4b can go in (change as needed)
                break;
            case LOW:
                target = 11.0;
                break;
            case MID:
                target = 18.0;
                break;
            case HIGH:
                target = 24.0;
                break;
            case STACK:
                target = 5.0 + height*(15.4/25.4);
                break;
        }
    }

    public void updatePID() {
        double error = target - encoderTicksToInches(m1.getCurrentPosition());
        double pid = error*kP + Math.copySign(kF, error);

        if(target == 0 && encoderTicksToInches(m1.getCurrentPosition()) < 0.2) {
            pid = 0;
        }

        m1.setPower(pid);
        m2.setPower(pid);
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double getCurrentPosition() {
        return encoderTicksToInches(m1.getCurrentPosition());
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
