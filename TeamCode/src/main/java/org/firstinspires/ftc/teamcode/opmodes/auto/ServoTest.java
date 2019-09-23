package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;
import org.firstinspires.ftc.teamcode.subsystems.Hook;

@Config
@TeleOp(name = "Servo Test", group = "test")
public class ServoTest extends AlmondLinear {

    public static double POSITION = 0.5;

    public void runOpMode() throws InterruptedException{

        hook = new Hook(hardwareMap);

        waitForStart();

        while(isStarted()&&!isStopRequested()){
            hook.setPosition(POSITION);
        }
    }
}
