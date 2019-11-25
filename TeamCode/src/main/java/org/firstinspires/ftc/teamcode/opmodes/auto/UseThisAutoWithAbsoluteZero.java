package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Hook;

@Autonomous()
public class UseThisAutoWithAbsoluteZero extends AlmondLinear {

    public void runOpMode() throws InterruptedException{

        drive = new DriveTrain(hardwareMap);
        claw = new Claw(hardwareMap);
        hook = new Hook(hardwareMap);

        hook.retract();
        claw.retract();

        waitForStart();

        back(24);

        ElapsedTime totalTime = new ElapsedTime();

        while(totalTime.milliseconds()<26000){
            drive.updatePoseEstimate();
        }

        driveSideways(0.5,500);
        turn(0);
        forward(60);
    }

}
