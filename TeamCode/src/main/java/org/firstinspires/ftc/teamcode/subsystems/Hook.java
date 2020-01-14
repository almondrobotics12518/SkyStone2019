package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Hook {

    private Servo hookLeft;
    private Servo hookRight;

    public static double RIGHT_OPEN_POS = 0;
    public static double LEFT_OPEN_POS = 0;

    public static double RIGHT_CLOSE_POS = 0;
    public static double LEFT_CLOSE_POS = 0;

    private boolean isClosed;

    public Hook(HardwareMap hardwareMap){
        hookLeft = hardwareMap.servo.get("hookLeft");
        hookRight = hardwareMap.servo.get("hookRight");

        open();
    }

    public void open(){
        hookRight.setPosition(RIGHT_OPEN_POS);
        hookLeft.setPosition(LEFT_OPEN_POS);
    }

    public void close(){
        hookRight.setPosition(RIGHT_CLOSE_POS);
        hookLeft.setPosition(LEFT_CLOSE_POS);
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
