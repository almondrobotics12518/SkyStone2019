package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "teleop",name="Tele Op")
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        boolean aWasPressed = false;

        DcMotor arm = hardwareMap.get(DcMotor.class,"arm");
        DcMotor wrist = hardwareMap.get(DcMotor.class,"wrist");

        Intake intake = new Intake(hardwareMap);

        MecanumDrive drive = new DriveTrain(hardwareMap);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(!isStopRequested()&&isStarted()){

            drive.setDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            if(gamepad2.a && !aWasPressed){
                aWasPressed = true;
                intake.toggle();
            }
            if(!gamepad2.a){
                aWasPressed = false;
            }



            arm.setPower(-gamepad2.right_stick_y*0.5);
            wrist.setPower(-gamepad2.left_stick_y*0.5);

            drive.updatePoseEstimate();


        }
    }

}
