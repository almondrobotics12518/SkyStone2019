package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.BrokenBarrierException;

public class Intake {

    DcMotor intakeLeft;
    DcMotor intakeRight;
    public CRServo pushIn;

    public Intake(HardwareMap  hardwareMap) {
            intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
            intakeRight = hardwareMap.dcMotor.get("intakeRight");
            pushIn = hardwareMap.crservo.get("pushIn");

            intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setPower(double power) {
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }

    public void setPushPower(double power){
        pushIn.setPower(power);
    }


}
