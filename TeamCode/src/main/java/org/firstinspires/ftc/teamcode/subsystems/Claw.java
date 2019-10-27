package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubEx;

@Config
public class Claw {

    public static double LEFT_EXTEND_POSITION = 0.15;
    public static double LEFT_RETRACT_POSITION = 0.55;

    public static double RIGHT_RETRACT_POSITION = 0.7;
    public static double RIGHT_EXTEND_POSITION = 1;

    public Servo clawLeft;
    public Servo clawRight;

    public Claw(HardwareMap hardwareMap){
        clawLeft = hardwareMap.get(Servo.class,"clawLeft");
        clawRight = hardwareMap.get(Servo.class,"clawRight");
    }

    public void extend(){
        clawLeft.setPosition(LEFT_EXTEND_POSITION);
        clawRight.setPosition(RIGHT_EXTEND_POSITION);
    }

    public void retract(){
        clawLeft.setPosition(RIGHT_RETRACT_POSITION);
        clawRight.setPosition(LEFT_RETRACT_POSITION);
    }
}
