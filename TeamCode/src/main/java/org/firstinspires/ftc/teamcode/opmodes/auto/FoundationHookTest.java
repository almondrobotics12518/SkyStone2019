package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.intellij.lang.annotations.JdkConstants;

@TeleOp(group="test",name="foundation servo test")
public class FoundationHookTest extends LinearOpMode {
    public Claw claw;
    public void runOpMode() throws InterruptedException {

        claw = new Claw(hardwareMap);

        waitForStart();

        while(isStarted() && !isStopRequested()){
            claw.extend();
        }
    }
}
