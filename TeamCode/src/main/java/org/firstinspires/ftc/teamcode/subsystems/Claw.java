package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw {

    private static double RETRACT_POSITION = 0;
    private static double EXTEND_POSITION = 1;

    public Servo claw;

    public Claw(HardwareMap hardwareMap){
        claw = hardwareMap.servo.get("clawServo");
    }

    public void setPosition(double position){
        claw.setPosition(position);
    }

    public void extend(){
        claw.setPosition(EXTEND_POSITION);
    }

    public void retract(){
        claw.setPosition(RETRACT_POSITION);
    }
}
