package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
@TeleOp(name="Intake Servo Test", group  = "test")
public class IntakeTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException{
        Intake intake = new Intake(hardwareMap);

        waitForStart();

        while(!isStopRequested()&&isStarted()){
            intake.close();
        }

    }
}
