package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw {

    public static double RETRACT_POSITION = 0;
    public static double EXTEND_POSITION = 1;

    public CRServo spin;
    public CRServo grab;

    public Claw(HardwareMap hardwareMap){
        spin = hardwareMap.crservo.get("spinServo");
        grab = hardwareMap.crservo.get("grabServo");
    }


    public void setSpinPower(double power){
        spin.setPower(power);
    }


    public void setGrabPower(double power){
        grab.setPower(power);
    }
}
