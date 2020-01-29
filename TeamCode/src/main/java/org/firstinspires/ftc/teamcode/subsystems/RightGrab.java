package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RightGrab {

    private Servo rightGrab;

    private double OPEN_POS = 0.5; // middle
    private double CLOSE_POS = .8;


    public RightGrab(HardwareMap hardwareMap){

        rightGrab = hardwareMap.servo.get("rightGrab");

        open();


    }

    public void open(){
        rightGrab.setPosition(OPEN_POS);
    }

    public void close(){
        rightGrab.setPosition(CLOSE_POS);
    }



}
