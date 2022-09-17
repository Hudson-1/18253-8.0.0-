package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.subclasses.VelocityPIDFController;

public class Shooter implements Subsystem {
    private boolean isTwoMotor;
    DcMotorEx m1;
    DcMotorEx m2;
    private double rpm;

    public static double GEAR_RATIO = 1.0 / 1.0; // output
    public static double TICKS_PER_SECOND = GEAR_RATIO * 28.0 / 60.0;

    public static double kV = 0;
    public static double kA = 0.0;
    public static double kStatic = 0;

    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.00125, 0, 0);
    public final VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
    private double lastTargetVelo;
    private ElapsedTime veloTimer;
    Gamepad g;


    public Shooter(Gamepad g, boolean isTwoMotor) {
        this.isTwoMotor = isTwoMotor;
        lastTargetVelo = 0;
        veloTimer = new ElapsedTime();
        rpm = 0;
        this.g = g;
    }

    @Override
    public void init(HardwareMap map) {
        m1 = map.get(DcMotorEx.class, "m1");
        if(isTwoMotor)
            m2 = map.get(DcMotorEx.class, "m2");

//        m1.setDirection(DcMotorSimple.Direction.REVERSE);
//        m2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void update() {
        if(!isTwoMotor) {
            updateOneMotor();
        } else {
            updateTwoMotor();
        }
    }

    public void updateOneMotor() {

    }

    public void updateTwoMotor() {
        double targetVelo = rpm * TICKS_PER_SECOND;

        // Call necessary controller methods
        veloController.setTargetVelocity(targetVelo);
        veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
        veloTimer.reset();

        lastTargetVelo = targetVelo;

        // Get the velocity from the motor with the encoder
        double motorVelo = m1.getVelocity();

        // Update the controller and set the power for each motor
        double power = veloController.update(m1.getCurrentPosition(), motorVelo);
        m1.setPower(power);
        m2.setPower(power);
    }

    public void setRpm(double rpm) {
        this.rpm = rpm;
    }

    @Override
    public Telemetry telemetry() {
        return null;
    }
}
