package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.LiftExt;

public class LiftPositionTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        LiftExt lift = new LiftExt(this);

        waitForStart();

        while(!isStopRequested()){
            lift.setPower(-gamepad1.left_stick_y);

            telemetry.addData("Lift Height: ",lift.getCurrentHeight());
            telemetry.update();

        }
    }
}
