package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LiftExt;

@TeleOp()
public class LiftTest extends LinearOpMode {


    public void runOpMode() throws InterruptedException{
        LiftExt lift = new LiftExt(this);
        waitForStart();
        lift.setHeight(10);
        while(!isStopRequested()){
            lift.update();
        }
    }

}
