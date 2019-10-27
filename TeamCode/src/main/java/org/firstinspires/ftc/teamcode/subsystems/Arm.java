package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.RevBulkData;

@Config
public class Arm {

    public static PIDCoefficients coefficients = new PIDCoefficients(0.1,0,0);
    public static double gravityFeedForward = 0;

    public static double GEAR_RATIO = 2;
    public static double TICKS_PER_REV = 1680;

    private double offset;
    private double targetPosition;
    private PIDFController controller;
    private double currentAngle;

    private Mode mode;
    private DcMotor motor;

    public enum Mode {
        IDLE,
        PID
    }

    public Arm(DcMotor motor, double offset, double gravityFeedForward){
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mode = Mode.IDLE;
        controller = new PIDFController(coefficients,1);
        this.offset = offset;
        this.gravityFeedForward = gravityFeedForward;
    }

    public double encoderTicksToRadians(int ticks){
        return (2 * Math.PI * GEAR_RATIO * ticks + offset)  / TICKS_PER_REV;
    }
    public void setAnglePID(double angle){
        controller.setTargetPosition(angle);
    }

    public void update(RevBulkData bulkData){
        currentAngle = encoderTicksToRadians(bulkData.getMotorCurrentPosition(motor));
        if(mode == Mode.PID){
            double power = controller.update(currentAngle,gravityFeedForward * Math.cos(currentAngle));
            setPower(power);
        }
    }

    public void setMode(Mode mode){this.mode = mode;}

    public double getCurrentAngle(){ return currentAngle; }

    public void setPower(double power){
        motor.setPower(power);

    }


}
