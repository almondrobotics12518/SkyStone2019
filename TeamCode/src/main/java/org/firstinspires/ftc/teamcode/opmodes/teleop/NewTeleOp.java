package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Hook;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LiftExt;
import org.firstinspires.ftc.teamcode.subsystems.LeftGrab;
import org.firstinspires.ftc.teamcode.subsystems.RightGrab;


@TeleOp(name="New TeleOp",group="teleop")
public class NewTeleOp extends LinearOpMode {

    private Claw claw;
    private DriveTrain dt;
    private Hook hook;
    private Intake intake;
    private RightGrab rightGrab;
    private LiftExt lift;
    private LeftGrab leftGrab;


    private boolean x2WasPressed = false;
    private boolean y2WasPressed = false;
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;
    private boolean xWasPressed = false;
    private boolean yWasPressed = false;
    private boolean a2WasPressed = false;


    @Override
    public void runOpMode() throws InterruptedException {

        leftGrab = new LeftGrab(hardwareMap);
        claw = new Claw(hardwareMap);
        hook = new Hook(hardwareMap);
        dt = new DriveTrain(hardwareMap);
        intake = new Intake(this);
        lift = new LiftExt(hardwareMap);
        rightGrab = new RightGrab(hardwareMap);


        leftGrab.open();
        leftGrab.retract();
        double multiplier=0.5;
        waitForStart();

        while (opModeIsActive()) {



            if(gamepad1.right_bumper){
                multiplier = 1;
            } else {
                multiplier = 0.5;
            }

            // drivetrain
            dt.setDrivePower(new Pose2d(gamepad1.left_stick_y*multiplier, gamepad1.left_stick_x*multiplier, -gamepad1.right_stick_x * 0.85 * multiplier));

            // slide


            // lift
            lift.setPower(-gamepad2.left_stick_y);

            // intake
            intake.setPower(gamepad2.right_trigger-gamepad2.left_trigger);

            // hook
            if(gamepad1.left_bumper && !aWasPressed){
                aWasPressed = true;
                hook.toggle();
            }
            if(!gamepad1.left_bumper){
                aWasPressed = false;
            }

            if(gamepad2.b && !bWasPressed){
                bWasPressed = true;
                claw.toggle();
            }
            if(!gamepad2.b){
                bWasPressed = false;
            }

            // claw

            if(gamepad2.x && !xWasPressed){
                xWasPressed = true;
                rightGrab.toggleSmall();
            }
            if(!gamepad2.x){
                xWasPressed = false;
            }


            if(gamepad2.y && !yWasPressed){
                yWasPressed = true;
                rightGrab.toggleBig();
            }
            if(!gamepad2.y){
                yWasPressed = false;
            }

            if(gamepad2.a && !a2WasPressed){
                a2WasPressed = true;
                lift.toggleCrank();
            }
            if(!gamepad2.a){
                a2WasPressed = false;
            }

            if(gamepad1.x && !x2WasPressed){
                x2WasPressed = true;
                leftGrab.toggleSmall();
            }
            if(!gamepad1.x){
                x2WasPressed = false;
            }

            if(gamepad1.y && !y2WasPressed){
                y2WasPressed = true;
                leftGrab.toggleBig();
            }
            if(!gamepad1.y){
                y2WasPressed = false;
            }



            claw.repeat();

            telemetry.addData("Wheels",dt.getWheelPositions());
            //telemetry.addData("Lift",lift.getCurrentHeight());
            telemetry.addData("Gamepad2 stick: ",gamepad2.left_stick_y);
            telemetry.update();


        }
    }
}
