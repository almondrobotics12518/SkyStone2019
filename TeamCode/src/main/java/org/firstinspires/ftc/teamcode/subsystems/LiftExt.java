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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubEx;

@Config
public class LiftExt {

    public static double RETRACT_POS = 1;
    public static double EXTEND_POS = 0.5;

    public static PIDCoefficients VELOCITY_PID = new PIDCoefficients(25,0,1);
    public static PIDCoefficients PID = new PIDCoefficients(0.2,0,0.02);
    public static double MAX_RPM = 312;
    public static double GRAVITY_FF = 0.12;

    public static double lastPower = 0;

    public static double LIFT_STAGES = 1;

    public static boolean isExtended = false;

    public static double maxPos = 36;
    public static double maxVel = 15;
    public static double maxAccel = 30;
    public static double maxJerk = 60;

    public static double kV = 0.046;
    public static double kA = 0.00009;
    public static double kStatic = 0.003;

    public static double TICKS_PER_REV = 537.6;
    public static double SPOOL_RADIUS = 0.75;

    public double lastTimeStamp = 0;

    private PIDFController controller;
    private DcMotorEx lift;
    public Servo crank;
    private ExpansionHubEx hub;
    private MotionProfile profile;
    private NanoClock clock = NanoClock.system();
    private double profileStartTime, desiredHeight = 0;
    private int offset;

    private MotionState state;

    public LiftExt(HardwareMap hardwareMap){
        hub = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 2");

        lift = hardwareMap.get(DcMotorEx.class, "verticalLift");
        crank = hardwareMap.get(Servo.class, "crank");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        lift.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                VELOCITY_PID.kP,VELOCITY_PID.kI,VELOCITY_PID.kD,1
        ));

        //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controller = new PIDFController(PID,kV,kA,kStatic);
        offset = lift.getCurrentPosition();

        retract();

        lastTimeStamp = clock.seconds();
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
            state = profile.get(time);
            controller.setTargetPosition(state.getX());
            power = controller.update(height, state.getV(), state.getA());

        } else {
            controller.setTargetPosition(desiredHeight);
            power = controller.update(height);
        }
        setPower(power);
    }

    public double getCurrentHeight(){
        return encoderTicksToInches(lift.getCurrentPosition()-offset);
    }

    public static double encoderTicksToInches(double ticks){
        return SPOOL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public void setPower(double power){
        lift.setPower(power + GRAVITY_FF);
    }

    public void setCrankPos(double position){
        crank.setPosition(position);
    }

    public void setMode(DcMotor.RunMode mode){
        lift.setMode(mode);
    }

    public static double getMaxRpm(){
        return MAX_RPM;
    }

    public MotionState getDesiredMotionState(){
        return state;
    }

    public static double rpmToVelocity(double rpm){
        return rpm * 2 * Math.PI * SPOOL_RADIUS;
    }

    public void resetHeight(){
        offset = lift.getCurrentPosition();
    }

    public void extend() {
        crank.setPosition(EXTEND_POS);
    }

    public void retract() {
        crank.setPosition(RETRACT_POS);
    }

    public void toggleCrank(){
        if(isExtended){
            retract();
            isExtended = false;
        } else {
            extend();
            isExtended = true;
        }
    }
}
