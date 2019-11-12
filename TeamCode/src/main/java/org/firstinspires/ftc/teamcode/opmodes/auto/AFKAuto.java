package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@Autonomous(name="AFK Auto",group="auto")
public class AFKAuto extends AlmondLinear {

    public void runOpMode() throws InterruptedException {
        drive = new DriveTrain(hardwareMap);

        waitForStart();

        forward(30);
    }
}
