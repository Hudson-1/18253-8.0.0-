package org.firstinspires.ftc.teamcode.subsystems;

import android.widget.Button;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BensMagic.AsymmetricProfile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.BensMagic.AsymmetricProfile.MotionConstraint;
import org.firstinspires.ftc.teamcode.util.ButtonPress;

@Config
public class Lift implements Subsystem {
    private boolean isTwoMotor;
    DcMotorEx m1;
    DcMotorEx m2;
    Gamepad g;

    public static double REST_slides = 0.0;
    public static double CHECK_slides = 4.0;
    public static double LOW_slides = 7.5;
    public static double MID_slides = 17.5;
    public static double HIGH_slides = 27;
    public static double STACK_slides = 9;

    public static double SPOOL_SIZE_IN = 0.5; // radius in inches
    public static double MOTOR_RATIO = 3.7;
    public static double TICKS_PER_REV = MOTOR_RATIO * 28.0;
    public static double GEAR_RATIO = 1;

    public static double kP = 0.12;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.14;
    private double target;

    public static double DECENT_POWER_MAX = 0.5;

    // this is the time we wait for the claw to close before moving.
    public static double WAIT_FOR_CLAW_MILLISECONDS = 200;
    // when putting back in, we wait this amount of time after we start moving the v4b
    public static double WAIT_FOR_V4B_IN = 500;
    public static double WAIT_FOR_CLAW_OPEN = 500;
    // change claw

    public static double clawOpen = 0.8;
    public static double clawClose = .95;

    // change v4b

    public static double rest = .65;
    public static double front = .9;
    public static double back = 0.165;
    public static double stack = .75;
    public static double front5 = .82;
    public static double front4 = .78;
    public static double front3 = .74;
    public static double front2 = .7;
    public static double front1 = .66;
    public static double intakein = .4;
    public static double intakeout = .07;
    boolean doResetClaw = true;

    Servo v4bL;
    Servo v4bR;
    Servo claw;
    Servo SIL;
    Servo SIR;




    public enum States {
        REST,
        LOW,
        MID,
        HIGH,
        LOW_ALTERNATIVE,
        MID_ALTERNATIVE,
        HIGH_ALTERNATIVE,
        STACK,
        STACK_1,
        STACK_2,
        STACK_3,
        STACK_4,
        STACK_5,
        STACK_SAFE,
        STACK_DEPOSIT,

    }

    public enum LiftState {
        REST,
        LOW,
        MID,
        HIGH,
        STACK,
        STACK_1,
        STACK_2,
        STACK_3,
        STACK_4,
        STACK_5,
        STACK_SAFE,
        CHECK
    }

    ElapsedTime timer;

    States state = States.REST;

    public boolean isAuto;

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
        SIL = map.get(Servo.class, "SIL");
        SIR = map.get(Servo.class, "SIR");
        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        //  m2.setDirection(DcMotorSimple.Direction.REVERSE);

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetServos(true);
    }

    public void resetServos(boolean doClaw) {
        v4bL.setPosition(1-rest);
        v4bR.setPosition(rest);

        if (doClaw) {
            if (isAuto) {
                claw.setPosition(clawClose);
            } else {
                claw.setPosition(clawOpen);
            }
        }

    }

    public void reset() {
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    ButtonPress upDpadPress = new ButtonPress();
    ButtonPress downDpadPress = new ButtonPress();
    ButtonPress leftBumperPress = new ButtonPress();

    @Override
    public void update() {

        if (isAuto) {
            autoUpdate();
        } else {
            teleopUpdate();
        }


    }


    public void autoUpdate() {
        updatePID();
    }

    public void teleopUpdate() {
        upDpadPress.button(g.dpad_up);
        downDpadPress.button(g.dpad_down);
        leftBumperPress.button(g.left_bumper);

        updatePID();
        System.out.println("state: " + state);


        switch(state) {
            case REST:
                // claw.setPosition(clawOpen); // preemptively open claw

                // after the timer has run enough, it will call reset servos and put the v4b back in
                if (timer.milliseconds() > WAIT_FOR_CLAW_OPEN) {
                    resetServos(doResetClaw);
                    intakein();
                    doResetClaw = false;
                    if (timer.milliseconds() > WAIT_FOR_V4B_IN + WAIT_FOR_CLAW_MILLISECONDS) {
                        if(timer.milliseconds() < 600 + WAIT_FOR_CLAW_MILLISECONDS) {
                            setLiftPosition(LiftState.CHECK, 0);
                        } else {
                            setLiftPosition(LiftState.REST, 0);
                        }
                    }
                }


                if (g.square) {
                    claw.setPosition(clawClose);
                }
                if (g.triangle) {
                    claw.setPosition(clawOpen);
                }

//                if (leftBumperPress.press()) {
//                    if (claw.getPosition() == clawClose) {
//                        claw.setPosition(clawOpen);
//                    } else {
//                        claw.setPosition(clawClose);
//                    }
//                }

                if(g.a) {
                    state = States.LOW;
                    grab();
                    intakeout();
                    timer.reset();
                    doResetClaw = true;
                }

                if(g.b) {
                    state = States.MID;
                    grab();
                    intakeout();
                    timer.reset();
                    doResetClaw = true;

                }

                if(g.right_bumper) {
                    state = States.HIGH;
                    grab();
                    intakeout();
                    timer.reset();
                    doResetClaw = true;

                }

                if(g.dpad_up || g.dpad_down) {
                    state = States.STACK_5;
                    claw.setPosition(clawOpen); // backwards nvm
                    doResetClaw = true;
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
                    claw.setPosition(clawOpen);
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
                    claw.setPosition(clawOpen);
                    state = States.REST;
                }
                break;
            case HIGH:
                if (timer.milliseconds() > WAIT_FOR_CLAW_MILLISECONDS) {
                    setLiftPosition(LiftState.HIGH, 5);
                    if(getCurrentPosition() > 4.0) {
                        back();
                    }
                    else if (getCurrentPosition() > 5.0) {
                    }
                }
                grab();
                if(g.left_bumper) {
                    timer.reset();
                    claw.setPosition(clawOpen);
                    state = States.REST;
                }
                break;
            case LOW_ALTERNATIVE:
                back();

                setLiftPosition(LiftState.LOW, 5);
                grab();

                if(g.left_bumper) {
                    timer.reset();
                    release(); // opposite day
                    state = States.REST;
                }
                break;
            case MID_ALTERNATIVE:
                back();

                setLiftPosition(LiftState.MID, 5);
                grab();

                if(g.left_bumper) {
                    timer.reset();
                    release(); // opposite day
                    state = States.REST;
                }
                break;
            case HIGH_ALTERNATIVE:
                back();

                setLiftPosition(LiftState.HIGH, 5);
                grab(); // reversed
                if(g.left_bumper) {
                    timer.reset();
                    release(); // opposite day
                    state = States.REST;
                }
                break;

            case STACK_1:
            case STACK_2:
            case STACK_3:
            case STACK_4:
            case STACK_5:
                intakeout();

                if (upDpadPress.press()) {
                    state = nextInStack(state);
                }
                if (downDpadPress.press()) {
                    state = previousInStack(state);
                }
                if (g.dpad_left) {
                    claw.setPosition(clawClose);
                }
                if (g.dpad_right) {
                    state = States.STACK_SAFE;
                }
                setV4bFromState(state);
                setLiftPosition(LiftState.REST,0);
                if(g.a) {
                    state = States.LOW;
                    release();
                    timer.reset();
                }

                if(g.b) {
                    state = States.MID;
                    release();
                    timer.reset();
                }

                if(g.right_bumper) {
                    state = States.HIGH;
                    release();
                    timer.reset();
                }
                break;
            case STACK_DEPOSIT:
                if (timer.milliseconds() > WAIT_FOR_CLAW_MILLISECONDS) {
                    setLiftPosition(LiftState.HIGH, 28.5);
                    if(getCurrentPosition() > 4){
                        front();
                    }
                }
                break;
        }

    }


    public void grab() {
        claw.setPosition(clawClose);
    }


    public void release() {
        claw.setPosition(clawOpen);
    }

    public void intakein() {
        SIL.setPosition(1-intakein);
        SIR.setPosition(intakein);

    }

    public void intakeout() {
        SIL.setPosition(1-intakeout);
        SIR.setPosition(intakeout);

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
    public void stack() {
        v4bL.setPosition(1-stack);
        v4bR.setPosition(stack);
    }

    public void setV4bFromState(States state) {
        double position = stateToVirtual4bar(state);
        v4bL.setPosition(1 - position);
        v4bR.setPosition(position);
    }

    public void front5() {
        v4bL.setPosition(1 - front5);
        v4bR.setPosition(front5);
    }
    public void front4() {
        v4bL.setPosition(1 - front4);
        v4bR.setPosition(front4);
    }
    public void front3() {
        v4bL.setPosition(1 - front3);
        v4bR.setPosition(front3);
    }
    public void front2() {
        v4bL.setPosition(1 - front2);
        v4bR.setPosition(front2);
    }
    public void front1() {
        v4bL.setPosition(1 - front1);
        v4bR.setPosition(front1);
    }


    public void slidesHigh() {
        setLiftPosition(LiftState.HIGH,0);
    }


    public void setLiftPosition(LiftState ls, double height) {
        switch(ls) {
            case REST:
                target = REST_slides;
                break;
            case STACK_1:
            case STACK_2:
            case STACK_3:
            case STACK_4:
            case STACK_5:
            case STACK_SAFE:
                target = stackHeightFromStatesForSlides(oppositeStateConversionForStack(ls));
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
                target = 5.0 + height*(15.4/25.4);
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

        if (target_local < CHECK_slides + 1.0) {
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


    public double stateToVirtual4bar(States stackState) {
        switch (stackState) {
            case STACK_1:
                return front1;
            case STACK_2:
                return front2;
            case STACK_3:
                return front3;
            case STACK_4:
                return front4;
            case STACK_5:
                return front5;
            default:
                return front1;
        }
    }

    public States nextInStack(States stackState) {
        switch (stackState) {
            case STACK_1:
                return States.STACK_2;
            case STACK_2:
                return States.STACK_3;
            case STACK_3:
                return States.STACK_4;
            case STACK_4:
            case STACK_5:
                return States.STACK_5;
            default:
                return States.STACK_1;
        }
    }
    public States previousInStack(States stackState) {
        switch (stackState) {
            case STACK_3:
                return States.STACK_2;
            case STACK_4:
                return States.STACK_3;
            case STACK_5:
                return States.STACK_4;
            case STACK_1:
            case STACK_2:
            default:
                return States.STACK_1;
        }
    }

    public double stackHeightFromStatesForSlides(States stackStates) {
        switch (stackStates) {
            case STACK_1:
                return .1;
            case STACK_2:
                return 2.1;
            case STACK_3:
                return 3.3;
            case STACK_4:
                return 4.5;
            case STACK_5:
                return 6;
            case STACK_SAFE:
                return 10;
        }
        return 0;
    }


    public LiftState stateConversionForStack(States state) {
        switch (state) {
            case STACK_1:
                return LiftState.STACK_1;
            case STACK_2:
                return LiftState.STACK_2;
            case STACK_3:
                return LiftState.STACK_3;
            case STACK_4:
                return LiftState.STACK_4;
            case STACK_SAFE:
                return LiftState.STACK_SAFE;
            default:
                return LiftState.STACK_5;
        }
    }
    public States oppositeStateConversionForStack(LiftState states) {
        switch (states) {
            case STACK_1:
                return States.STACK_1;
            case STACK_2:
                return States.STACK_2;
            case STACK_3:
                return States.STACK_3;
            case STACK_4:
                return States.STACK_4;
            case STACK_SAFE:
                return States.STACK_SAFE;
            default:
                return States.STACK_5;
        }
    }


    @Override
    public Telemetry telemetry() {
        return null;
    }
}