package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class StickServo implements Subsystem {
    public static double start = 0;
    public static double end = 1;
    private Gamepad g;
    private String configName;
    public static double speedDividor = 20.0;
    double lastPosition;

    boolean lastPress = false;
    boolean toggle = false;

    Servo servo;

    public StickServo(Gamepad g, String name) {
        this.g = g;
        this.configName = name;
        lastPosition = 0;
    }

    @Override
    public void init(HardwareMap map) {
        servo = map.get(Servo.class, configName);
        servo.setPosition(lastPosition);
    }

    @Override
    public void update() {

    }

    public void in() {
        servo.setPosition(start);
    }

    public void out() {
        servo.setPosition(end);
    }

    public void stickControl(double stick) {
        lastPosition += stick / speedDividor;
        servo.setPosition(lastPosition);
    }

    @Override
    public Telemetry telemetry() {
        return null;
    }
}
