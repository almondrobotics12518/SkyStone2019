package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    boolean isClamped = false;

    public Servo turn;
    public Servo intake;

    public static double OPEN_POS = 0.5;
    public static double CLOSE_POS = 0;

    public Intake(HardwareMap hardwareMap){
        turn = hardwareMap.get(Servo.class,"turn");
        intake = hardwareMap.get(Servo.class,"intake");
    }



    public void setIntakePosition(double position) { intake.setPosition(position); }

    public void clamp(){
        intake.setPosition(CLOSE_POS);
    }

    public void open(){
        intake.setPosition(OPEN_POS);
    }

    public void setTurnPosition(double position){ turn.setPosition(position); }

    public void toggle(){
        if(isClamped){
            isClamped = false;
            open();
        } else {
            isClamped = true;
            clamp();
        }
    }

}


