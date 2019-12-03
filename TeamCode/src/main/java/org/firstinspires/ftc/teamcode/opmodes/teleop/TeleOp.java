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

/*
    Tele op code for november qualifier
    This includes basic functionality to just set power to motors
    based on joystick input.
 */


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "teleop",name="Tele Op")
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        //These two booleans are used in order to store whether a button was pressed in the  last loop
        //This is useful for toggles and things based on a single button press

        boolean aWasPressed = false;
        boolean yWasZero = true;

        //Initialize hardware map for the arm and wrist motors (these two motors aren't in their own

        DcMotor arm = hardwareMap.get(DcMotor.class,"arm");
        DcMotor wrist = hardwareMap.get(DcMotor.class,"wrist");

        //Initializes the subsystems. The opmode's hardwaremap is passed into them

        Claw claw = new Claw(hardwareMap);

        Intake intake = new Intake(hardwareMap);

        MecanumDrive drive = new DriveTrain(hardwareMap);

        Hook hook = new Hook(hardwareMap);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        //This loop runs for all of teleop

        while(!isStopRequested()&&isStarted()){

            //This sets the power for each componenet of the robot's movement
            //x is forwards and y is sideways because it's easier to work with when using global field coordinates

            double yPower = gamepad1.left_stick_x*0.6;
            double xPower = -gamepad1.left_stick_y*0.4;
            double turnPower = gamepad1.right_stick_x*0.4;

            if(gamepad1.right_bumper){
                yPower *= 1.6;
                xPower *= 2.5;
                turnPower *= 2.5;
            }


            //add power normalization in the future
            drive.setMotorPowers(xPower+yPower+turnPower,xPower-yPower+turnPower,xPower+yPower-turnPower,xPower-yPower-turnPower);

            //this toggles the intake's position
            if(gamepad2.a && !aWasPressed){
                aWasPressed = true;
                intake.toggle();
            }
            if(!gamepad2.a){
                aWasPressed = false;
            }


            //This sets the arm's power based on joystick inputs
            arm.setPower(-gamepad2.right_stick_y*0.7);
            wrist.setPower(gamepad2.left_stick_y*0.5);


            //These all control servos and wil extend and retract them using different buttons
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
