package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RightGrab {

    private boolean isClosed;
    private boolean isRetracted;


    private Servo rightBig;
    private Servo rightSmall;

    private double OPEN_POS = 0.55;// middle
    private double CLOSE_POS = 0.1;

    private double RETRACT_POS = 0.45; // middle
    private double EXTEND_POS = 0.85;


    public RightGrab(HardwareMap hardwareMap){

        rightSmall = hardwareMap.servo.get("rightSmall");
        rightBig = hardwareMap.servo.get("rightBig");

        open();


    }

    public void setBigPosition(double position){
        rightBig.setPosition(position);
    }

    public void open(){
        rightSmall.setPosition(OPEN_POS);
    }

    public void close(){
        rightSmall.setPosition(CLOSE_POS);
    }

    public void retract(){
        rightBig.setPosition(RETRACT_POS);
    }

    public void extend(){
        rightBig.setPosition(EXTEND_POS);
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


}
