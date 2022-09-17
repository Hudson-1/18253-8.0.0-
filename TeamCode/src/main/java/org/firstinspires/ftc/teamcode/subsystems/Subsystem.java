package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Subsystem {
    void init(HardwareMap map);
    void update();
    Telemetry telemetry();


}
