package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LeftGrab {

    private boolean isClosed;
    private boolean isRetracted;

    private Servo leftBig;
    private Servo leftSmall;

    private double OPEN_POS = 0.15; // middle
    private double CLOSE_POS = .6;

    private double RETRACT_POS = 0.5; // middle
    private double EXTEND_POS = 0.05;


    public LeftGrab(HardwareMap hardwareMap){

        leftSmall = hardwareMap.servo.get("leftSmall");
        leftBig = hardwareMap.servo.get("leftBig");

        retract();
        open();


    }

    public void open(){
        leftSmall.setPosition(OPEN_POS);
    }

    public void close(){
        leftSmall.setPosition(CLOSE_POS);
    }

    public void retract(){
        leftBig.setPosition(RETRACT_POS);
    }

    public void extend(){
        leftBig.setPosition(EXTEND_POS);
    }

    public void toggleSmall(){
        if(isClosed){
            isClosed = false;
            open();
        }else {
            isClosed = true;
            close();
        }

    }
    public void toggleBig(){
        if(isRetracted){
            isRetracted = false;
            extend();
        }else {
            isRetracted = true;
            retract();
        }

    }

    public void setBigPosition(double position){
        leftBig.setPosition(position);
    }




}
