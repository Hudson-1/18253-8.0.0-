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

    public static double kP = 0.5;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;
    private double target;

    // this is the time we wait for the claw to close before moving.
    public static double WAIT_FOR_CLAW_MILLISECONDS = 800;
    // when putting back in, we wait this amount of time after we start moving the v4b
    public static double WAIT_FOR_V4B_IN = 400;

    // change claw

    public static double clawOpen = 0.3;
    public static double clawClose = 0;

    // change v4b

    public static double rest = 0.61;
    public static double front = 0.95;
    public static double back = 0.01;

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
      //  m2.setDirection(DcMotorSimple.Direction.REVERSE);

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetServos();
    }

    public void resetServos() {
        v4bL.setPosition(1-rest);
        v4bR.setPosition(rest);
        claw.setPosition(clawOpen);
    }

    public void reset() {
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void update() {
        updatePID();
        System.out.println("state: " + state);
        switch(state) {
            case REST:
                claw.setPosition(clawOpen); // preemptively open claw

                // after the timer has run enough, it will call reset servos and put the v4b back in
                if (timer.milliseconds() > WAIT_FOR_CLAW_MILLISECONDS) {
                    resetServos();
                    if (timer.milliseconds() > WAIT_FOR_V4B_IN + WAIT_FOR_CLAW_MILLISECONDS) {
                        if(timer.milliseconds() < 600 + WAIT_FOR_V4B_IN) {
                            setLiftPosition(LiftState.CHECK, 0);
                        } else {
                            setLiftPosition(LiftState.REST, 0);
                        }
                    }
                }


                if(g.a) {
                    state = States.LOW;
                    grab();
                    timer.reset();
                }

                if(g.b) {
                    state = States.MID;
                    grab();
                    timer.reset();
                }

                if(g.right_bumper) {
                    state = States.HIGH;
                    grab();
                    timer.reset();
                }

                break;
            case LOW:
                if (timer.milliseconds() > WAIT_FOR_CLAW_MILLISECONDS) {
                    setLiftPosition(LiftState.LOW, 0);
                    if(getCurrentPosition() > 4.0) {
                        back();
                    }
                }
                grab();
                if(g.left_bumper) {
                    timer.reset();
                    state = States.REST;
                }
                break;
            case MID:
                if (timer.milliseconds() > WAIT_FOR_CLAW_MILLISECONDS) {
                    setLiftPosition(LiftState.MID, 0);
                    if(getCurrentPosition() > 4.0) {
                        back();
                    }

                }
                grab();
                if(g.left_bumper) {
                    timer.reset();
                    state = States.REST;
                }
                break;
            case HIGH:
                if (timer.milliseconds() > WAIT_FOR_CLAW_MILLISECONDS) {
                    setLiftPosition(LiftState.HIGH, 5);
                    if(getCurrentPosition() > 4.0) {
                        back();
                    }
                }
                grab();
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
        v4bL.setPosition(1-back);
        v4bR.setPosition(back);

    }
    public void rest() {
        v4bL.setPosition(rest);
        v4bR.setPosition(1-rest);

    }

    public void front() {
        v4bL.setPosition(1-front);
        v4bR.setPosition(front);
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

        System.out.println("target is: " + target);
        System.out.println("lift pos is: " + encoderTicksToInches(m1.getCurrentPosition()));
        System.out.println("pid power: " + pid);

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