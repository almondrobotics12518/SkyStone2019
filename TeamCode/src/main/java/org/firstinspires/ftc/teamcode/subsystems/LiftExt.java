package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.openftc.revextensions2.ExpansionHubEx;

@Config
public class LiftExt {

    public static PIDCoefficients VELOCITY_PID = new PIDCoefficients(25,0,1);
    public static PIDCoefficients PID = new PIDCoefficients(0.03,0,0);
    public static double MAX_RPM = 312;

    public static double LIFT_STAGES = 1;

    public static double maxPos = 36;
    public static double maxVel = 20;
    public static double maxAccel = 15;
    public static double maxJerk = 0;

    public static double kV = 0.051;
    public static double kA = 0;
    public static double kStatic = 0;

    public static double TICKS_PER_REV = 537.6;
    public static double SPOOL_RADIUS = 0.75;

    private PIDFController controller;
    private DcMotorEx lift;
    private ExpansionHubEx hub;
    private MotionProfile profile;
    private NanoClock clock = NanoClock.system();
    private double profileStartTime, desiredHeight = 0;
    private int offset;

    public LiftExt(LinearOpMode opmode){
        hub = opmode.hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 2");

        lift = opmode.hardwareMap.get(DcMotorEx.class, "verticalLift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                VELOCITY_PID.kP,VELOCITY_PID.kI,VELOCITY_PID.kD,1
        ));

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDFController(PID,kV,kA,kStatic);
        offset = lift.getCurrentPosition();
    }

    public boolean isBusy() {
        return profile != null && (clock.seconds() - profileStartTime) <= profile.duration();
    }

    public void setHeight(double height) {
        height = Math.min(Math.max(0, height), maxPos);

        double time = clock.seconds() - profileStartTime;
        MotionState start = isBusy() ? profile.get(time) : new MotionState(desiredHeight, 0, 0, 0);
        MotionState goal = new MotionState(height, 0, 0, 0);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start, goal, maxVel, maxAccel, maxJerk
        );
        profileStartTime = clock.seconds();

        this.desiredHeight = height;
    }

    public void update(){
        double power;
        double height = getCurrentHeight();
        if(isBusy()){
            double time = clock.seconds() - profileStartTime;
            MotionState state = profile.get(time);
            controller.setTargetPosition(state.getX());
            power = controller.update(height, state.getV(), state.getA());
        } else {
            controller.setTargetPosition(desiredHeight);
            power = controller.update(height);
        }
        lift.setPower(power);
    }

    public double getCurrentHeight(){
        return encoderTicksToInches(lift.getCurrentPosition()-offset);
    }

    public static double encoderTicksToInches(double ticks){
        return SPOOL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public void setPower(double power){
        lift.setPower(power);
    }

    public void setMode(DcMotor.RunMode mode){
        lift.setMode(mode);
    }

    public static double getMaxRpm(){
        return MAX_RPM;
    }

    public static double rpmToVelocity(double rpm){
        return rpm * 2 * Math.PI * SPOOL_RADIUS;
    }
}
