package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    public Servo turn;
    public Servo intake;

    public double OPEN_POS = 0.5;
    public double CLOSE_POS = 0;

    public Intake(HardwareMap hardwareMap){
        turn = hardwareMap.get(Servo.class,"turn");
        intake = hardwareMap.get(Servo.class,"intake");
    }

    public void openIntake(){
        intake.setPosition(OPEN_POS);
    }

    public void closeIntake(){
        intake.setPosition(CLOSE_POS);
    }
}
