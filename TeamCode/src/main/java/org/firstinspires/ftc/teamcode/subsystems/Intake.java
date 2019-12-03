package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {

    boolean isClamped = false;

    public Servo intakeRight;
    public Servo intakeLeft;

    public static double RIGHT_OPEN_POS = 0.7;
    public static double RIGHT_CLOSE_POS = 0.48;

    public static double LEFT_OPEN_POS = 0.3;
    public static double LEFT_CLOSE_POS = 0.45;

    public Intake(HardwareMap hardwareMap){
        intakeRight = hardwareMap.get(Servo.class,"intakeRight");
        intakeLeft = hardwareMap.get(Servo.class,"intakeLeft");
    }



    public void setRightIntakePosition(double position) { intakeRight.setPosition(position); }

    public void setLeftIntakePosition(double position) { intakeLeft.setPosition(position);}

    public void close() {
        intakeRight.setPosition(RIGHT_CLOSE_POS);
        intakeLeft.setPosition(LEFT_CLOSE_POS);
        isClamped = true;
    }

    public void open() {
        intakeRight.setPosition(RIGHT_OPEN_POS);
        intakeLeft.setPosition(LEFT_OPEN_POS);
        isClamped = false;
    }

    public void toggle(){
        if(isClamped){
            isClamped = false;
            open();
        } else {
            isClamped = true;
            close();
        }
    }

}


