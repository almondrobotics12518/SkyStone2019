package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

import java.util.List;

@TeleOp(group="test",name="Vision Test")
public class VisionTest extends AlmondLinear {

    public Positions skystonePosition;

    public void runOpMode(){
        initVuforia();
        initTF();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                   autoPath();
            }
        }


    }



}
