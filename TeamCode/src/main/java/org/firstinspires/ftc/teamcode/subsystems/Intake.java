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

    private boolean stoneInside = false;
    public static double threshold = 1500;

    private DcMotorEx motorLeft;
    private DcMotorEx motorRight;

    public Intake(LinearOpMode opmode){
        motorLeft = opmode.hardwareMap.get(DcMotorEx.class,"intakeLeft");
        motorRight = opmode.hardwareMap.get(DcMotorEx.class, "intakeRight");

        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double leftPower, double rightPower){
        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
    }

    public void intake(){
        setPower(1,1);
        if(getVelocity()<threshold){
            setPower(0.5,0.5);
        }
    }
    
    public double getVelocity(){
        return motorLeft.getVelocity();
    }

    public void outtake(){
        setPower(-1,-1);
    }

}
