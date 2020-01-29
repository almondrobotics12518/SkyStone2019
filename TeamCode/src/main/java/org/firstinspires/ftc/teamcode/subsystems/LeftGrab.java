package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LeftGrab {

    private Servo leftGrab;

    private double OPEN_POS = 0.5; // middle
    private double CLOSE_POS = .8;


    public LeftGrab(HardwareMap hardwareMap){

        leftGrab = hardwareMap.servo.get("leftGrab");

        open();


    }

    public void open(){
        leftGrab.setPosition(OPEN_POS);
    }

    public void close(){
        leftGrab.setPosition(CLOSE_POS);
    }



}
