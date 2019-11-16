package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hook {
    private Servo hook;
    public static double RETRACT_POS = 0.9;
    public static double EXTEND_POS = 0.5;


    public Hook(HardwareMap hardwareMap){
        hook = hardwareMap.servo.get("hook");
    }

    public void setPosition(double position){
        hook.setPosition(position);
    }

    public void retract(){
        hook.setPosition(RETRACT_POS);
    }

    public void extend(){
        hook.setPosition(EXTEND_POS);
    }
}
