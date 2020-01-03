package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.revextensions2.ExpansionHubMotor;

public class Intake{
    /*
     * These motors should be left and right when viewing the robot from the back.
     * You should be facing the same way that the intake is facing.
     */

    private DcMotorEx motorLeft;
    private DcMotorEx motorRight;

    public Intake(LinearOpMode opmode){
        motorLeft = opmode.hardwareMap.get(DcMotorEx.class,"intakeLeft");
        motorRight = opmode.hardwareMap.get(DcMotorEx.class, "intakeRight");

        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power){
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    public void intake(){
        setPower(1);

    }


    public void outtake(){
        setPower(-1);
    }

}
