package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleTankDriveREVOptimized;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Hook;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ParallelogramLift;


@TeleOp(name="TeleOp",group="teleOp")
public class TeleOpNew extends AlmondLinear {


    public void runOpMode() throws InterruptedException{
        drive = new SampleTankDriveREVOptimized(hardwareMap);
        intake = new Intake(hardwareMap);
        hook = new Hook(hardwareMap);
        lift = new ParallelogramLift(hardwareMap);
        claw = new Claw(hardwareMap);


        waitForStart();

        hook.retract();
        lift.setPower(0);



        while(isStarted()&&!isStopRequested()){

            //drive power setting
            drive.setDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            lift.setPower(-gamepad2.right_stick_y);
            intake.setPower(gamepad2.left_trigger-gamepad2.right_trigger);

            claw.setGrabPower(gamepad2.left_stick_x);

            if(gamepad2.dpad_left){
                claw.setSpinPower(.7);
            }

            else if(gamepad2.dpad_right){
                claw.setSpinPower(-.7);
            }

            else{
                claw.setSpinPower(0);
            }

            if(gamepad2.a){
                hook.extend();
            }

            if(gamepad2.b){
                hook.retract();
            }


            if(gamepad2.left_bumper){
                intake.setPushPower(.7);
            }

            else if(gamepad2.right_bumper){
                intake.setPushPower(-.7);
            } else {
                intake.setPushPower(0);
            }

            drive.updatePoseEstimate();

        }



    }

}
