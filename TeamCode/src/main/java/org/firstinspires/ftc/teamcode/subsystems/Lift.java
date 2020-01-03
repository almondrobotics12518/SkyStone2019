package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private DcMotor lift;

    public static double FIRST_LEVEL = 2;
    public static double INCHES_PER_LEVEL = 4;
    public static double SPOOL_DIAMETER = 1.5;
    public static double TICKS_PER_REVOLUTION = 576.4;

    public Lift(HardwareMap hardwareMap){
        lift = hardwareMap.get(DcMotor.class,"verticalLift");

        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
    }

    public static double encoderTicksToInches(double ticks){
        return ticks * SPOOL_DIAMETER * 2 * Math.PI / TICKS_PER_REVOLUTION;
    }

    public double getCurrentPosition(){
        return encoderTicksToInches(lift.getCurrentPosition());
    }

    public void setTargetPosition(double inches){
        int target =(int)(inches * TICKS_PER_REVOLUTION / (SPOOL_DIAMETER * 2 * Math.PI));
        lift.setTargetPosition(target);
        lift.setPower(1);
    }

    public void setLevel(int level){
        setTargetPosition(level * INCHES_PER_LEVEL + FIRST_LEVEL);
    }
}
