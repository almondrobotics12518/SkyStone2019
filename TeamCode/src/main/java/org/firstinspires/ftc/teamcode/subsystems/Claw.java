package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw {

    private Servo claw;

    public static double OPEN_POS = 0;
    public static double CLOSE_POS = 0.5;
    private boolean isClosed = false;

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

    public void toggle(){
        if(isClosed){
            isClosed = false;
            open();
        }else {
            isClosed = true;
            close();
        }

    }


}
