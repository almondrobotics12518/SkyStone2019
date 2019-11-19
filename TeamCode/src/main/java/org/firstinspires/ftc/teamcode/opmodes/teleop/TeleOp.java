package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Hook;
import org.firstinspires.ftc.teamcode.subsystems.Intake;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "teleop",name="Tele Op")
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        boolean aWasPressed = false;
        boolean yWasZero = true;


        DcMotor arm = hardwareMap.get(DcMotor.class,"arm");
        DcMotor wrist = hardwareMap.get(DcMotor.class,"wrist");

        Claw claw = new Claw(hardwareMap);

        Intake intake = new Intake(hardwareMap);

        MecanumDrive drive = new DriveTrain(hardwareMap);

        Hook hook = new Hook(hardwareMap);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(!isStopRequested()&&isStarted()){

            double yPower = gamepad1.left_stick_x*0.6;
            double xPower = -gamepad1.left_stick_y*0.4;
            double turnPower = gamepad1.right_stick_x*0.4;

            if(gamepad1.right_bumper){
                yPower *= 1.6;
                xPower *= 2.5;
                turnPower *= 2.5;
            }



            drive.setMotorPowers(xPower+yPower+turnPower,xPower-yPower+turnPower,xPower+yPower-turnPower,xPower-yPower-turnPower);

            if(gamepad2.a && !aWasPressed){
                aWasPressed = true;
                intake.toggle();
            }
            if(!gamepad2.a){
                aWasPressed = false;
            }

            arm.setPower(-gamepad2.right_stick_y*0.7);
            wrist.setPower(gamepad2.left_stick_y*0.5);

            if(gamepad2.right_bumper){
                claw.retract();
            }

            if(gamepad2.left_bumper){
                claw.extend();
            }

            if(gamepad2.b){
                hook.extend();
            }
            if(gamepad2.x){
                hook.retract();
            }


        }
    }

}
