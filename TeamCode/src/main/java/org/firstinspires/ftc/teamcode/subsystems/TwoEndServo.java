package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class TwoEndServo implements Subsystem {
    public static double start = 0;
    public static double end = 1;
    private String configName;

    boolean lastPress = false;
    boolean toggle = false;

    Servo servo;

    public TwoEndServo(String name) {
        this.configName = name;
    }

    @Override
    public void init(HardwareMap map) {
        servo = map.get(Servo.class, configName);
    }

    @Override
    public void update() {
        //
    }

    public void in() {
        servo.setPosition(start);
    }

    public void out() {
        servo.setPosition(end);
    }

    public void toggle(boolean button) {
        if(button)
            lastPress = true;

        if(lastPress && !button) {
            toggle = !toggle;
            lastPress = false;
        }

        if(toggle)
            servo.setPosition(start);
        else
            servo.setPosition(end);
    }

    public void twoButton(boolean buttonStart, boolean buttonEnd) {
        if(buttonStart)
            servo.setPosition(start);

        if(buttonEnd)
            servo.setPosition(end);
    }

    @Override
    public Telemetry telemetry() {
        return null;
    }
}
