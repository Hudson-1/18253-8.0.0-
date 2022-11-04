package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BensMagic.AsymmetricProfile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.BensMagic.AsymmetricProfile.MotionConstraint;

@Config
public class Lift implements Subsystem {
    private boolean isTwoMotor;
    DcMotorEx m1;
    DcMotorEx m2;
    Gamepad g;

    public static double REST_slides = 0.0;
    public static double CHECK_slides = 4.0;
    public static double LOW_slides = 7;
    public static double MID_slides = 16;
    public static double HIGH_slides = 25.5;
    public static double STACK_slides = 9;

    public static double SPOOL_SIZE_IN = 0.5; // radius in inches
    public static double MOTOR_RATIO = 3.7;
    public static double TICKS_PER_REV = MOTOR_RATIO * 28.0;
    public static double GEAR_RATIO = 1;

    public static double kP = 0.8;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;
    private double target;

    public static double DECENT_POWER_MAX = 0.2;

    // this is the time we wait for the claw to close before moving.
    public static double WAIT_FOR_CLAW_MILLISECONDS = 500;
    // when putting back in, we wait this amount of time after we start moving the v4b
    public static double WAIT_FOR_V4B_IN = 600;
public static double WAIT_FOR_CLAW_OPEN = 700;
    // change claw

    public static double clawOpen = 0.66;
    public static double clawClose = .4;

    // change v4b

    public static double rest = .31;
    public static double front = 0;
    public static double back = 0.9;
    public static double stack = 0.7;


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

    LiftState liftState = LiftState.REST;

    ElapsedTime timer;

    States state = States.REST;

    boolean isAuto;

    public Lift(Gamepad g, boolean isTwoMotor, boolean isAuto) {
        this.g = g;
        this.isTwoMotor = isTwoMotor;
        target = 0;
        timer = new ElapsedTime();
        this.isAuto = isAuto;
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



        if (isAuto) {
            updateAuto();
        } else {
            updateTeleop();
        }

    }

    private void updateAuto() {
        updatePID();
    }


    boolean previousLB = false;

    public void updateTeleop() {
        System.out.println("state: " + state);
        boolean currentLB = g.left_bumper;
        boolean LBIsPressed = !previousLB && currentLB;
        previousLB = currentLB;

        updatePID();
        switch(state) {
            case REST:

                if (LBIsPressed) {
                    double currentPos = claw.getPosition();
                    if (Double.isNaN(currentPos)) {
                        claw.setPosition(clawClose);
                    }
                    if (currentPos == clawOpen) {
                        claw.setPosition(clawClose);
                    }
                    if (currentPos == clawClose) {
                        claw.setPosition(clawOpen);
                    }
                }

                // after the timer has run enough, it will call reset servos and put the v4b back in
                if (timer.milliseconds() > WAIT_FOR_CLAW_OPEN) {
                    resetServos();
                    if (timer.milliseconds() > WAIT_FOR_V4B_IN + WAIT_FOR_CLAW_MILLISECONDS) {
                        if(timer.milliseconds() < 600 + WAIT_FOR_CLAW_MILLISECONDS) {
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

                if(g.dpad_right) {
                    state = States.STACK;
                    grab();
                    timer.reset();
                }
                if(g.dpad_up) {
                    state = States.STACK_DEPOSIT;
                    claw.setPosition(clawOpen);
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
                setLiftPosition(LiftState.STACK, 0);
                if(getCurrentPosition() > 4){
                    stack();

                }
                break;
            case STACK_DEPOSIT:
                if (timer.milliseconds() > WAIT_FOR_CLAW_MILLISECONDS) {
                    setLiftPosition(LiftState.HIGH, 28);
                    if(getCurrentPosition() > 4){
                        front();
                    }
                }
                break;
        }
    }




    public void slidesHigh() {
        setLiftPosition(LiftState.HIGH,HIGH_slides);
    }

    public void slidesIn() {
        setLiftPosition(LiftState.REST,REST_slides);
    }

    public void grab() {
        claw.setPosition(clawClose);
    }

    public void release() {
        claw.setPosition(clawOpen);
    }

    public void back() {
        v4bL.setPosition(1-back);
        v4bR.setPosition(back);
    }
    public void front() {
        v4bL.setPosition(1-front);
        v4bR.setPosition(front);
    }

    public void slideStack(double heightIN) {
        setLiftPosition(LiftState.STACK,heightIN);
    }


    public void rest() {
        v4bL.setPosition(rest);
        v4bR.setPosition(1-rest);
    }

    public void stack() {
        v4bL.setPosition(1-stack);
        v4bR.setPosition(stack);
    }

    public void setLiftPosition(LiftState ls, double height) {
        liftState = ls;
        switch(ls) {
            case REST:
                target = REST_slides;
                break;
            case CHECK:
                target = CHECK_slides; // this is your minimum position where your v4b can go in (change as needed)
                break;
            case LOW:
                target = LOW_slides;
                break;
            case MID:
                target = MID_slides;
                break;
            case HIGH:
                target = HIGH_slides;
                break;
            case STACK:
                target = height;
                break;
        }
    }

    public void updatePID() {

        double target_local = target;


        double error1 = target_local - encoderTicksToInches(m1.getCurrentPosition());
        double pid1 = error1*kP + Math.copySign(kF, error1);

        double error2 = target_local - encoderTicksToInches(m2.getCurrentPosition());
        double pid2 = error2*kP + Math.copySign(kF, error2);
        if(target == 0 && encoderTicksToInches(m1.getCurrentPosition()) < 0.2) {
            pid1 = 0;
        }
        if(target == 0 && encoderTicksToInches(m2.getCurrentPosition()) < 0.2) {
            pid2 = 0;
        }

        if (target_local < CHECK_slides + 1.0 || liftState.equals(LiftState.STACK)) {
            pid1 = Range.clip(pid1,-DECENT_POWER_MAX,DECENT_POWER_MAX);
            pid2 = Range.clip(pid2,-DECENT_POWER_MAX,DECENT_POWER_MAX);
        }

        System.out.println("target is: " + target);
        System.out.println("lift pos is: " + encoderTicksToInches(m1.getCurrentPosition()));
        System.out.println("pid power: " + pid1);

        m1.setPower(pid1);
        m2.setPower(pid2);
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