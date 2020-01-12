package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Hook;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LiftExt;



@TeleOp(name="New TeleOp",group="teleop")
public class NewTeleOp extends LinearOpMode {

    private Claw claw;
    private DriveTrain dt;
    private Hook hook;
    private Intake intake;
    private LiftExt lift;
    private double currentTime;

    private double leftX = 0;
    private double leftY = 0;
    private double rightX = 0;
    private double LF = 0;
    private double LR = 0;
    private double RF = 0;
    private double RR = 0;

    private double rightMultiplier = 0;

    private boolean hookIsClosed = false;
    private boolean aWasPressed = false;


    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime timer = new ElapsedTime();
        claw = new Claw(hardwareMap);
        hook = new Hook(hardwareMap);
        dt = new DriveTrain(hardwareMap);
        intake = new Intake(this);
        lift = new LiftExt(this);

        waitForStart();
        timer.reset();
        while (opModeIsActive()) {
            currentTime = timer.milliseconds();

            dt.setDrivePower(new Pose2d(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x * 0.3));
            lift.setSlidePower(-gamepad1.left_stick_y*0.7);

            if(gamepad2.a && !aWasPressed){
                aWasPressed = true;
                if(hookIsClosed){
                    hook.open();
                } else {
                    hook.close();
                }
                hookIsClosed = !hookIsClosed;

            } else if(!gamepad2.a && aWasPressed){
                aWasPressed = false;
            }


        }
    }
}
