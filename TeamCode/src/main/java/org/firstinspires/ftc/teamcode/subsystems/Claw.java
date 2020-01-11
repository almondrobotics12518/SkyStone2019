package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    private Servo claw;

    public static double OPEN_POS;
    public static double CLOSE_POS;

    public Claw(HardwareMap hardwareMap){
        claw = hardwareMap.servo.get("claw");
    }

    public void setPosition(double position){
        claw.setPosition(position);
    }

    public void open(){
        claw.setPosition(OPEN_POS);
    }

    public void close(){
        claw.setPosition(CLOSE_POS);
    }


}
