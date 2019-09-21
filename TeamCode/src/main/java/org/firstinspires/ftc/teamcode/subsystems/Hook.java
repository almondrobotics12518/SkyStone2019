package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hook {
    private Servo hook;
    public static double RETRACT_POS = 0.3;
    public static double EXTEND_POS = 0.55;


    public Hook(HardwareMap hardwareMap){
        hook = hardwareMap.servo.get("Hook");
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
