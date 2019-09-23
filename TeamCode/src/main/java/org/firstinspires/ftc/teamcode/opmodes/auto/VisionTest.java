package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

@TeleOp(group="test",name="Vision Test")
public class VisionTest extends AlmondLinear {

    public Positions skystonePosition;

    public void runOpMode(){
        initVuforia();
        initTF();

        while(!isStarted()&&!isStopRequested()){
            skystonePosition = autoPath();

            telemetry.addLine(""+skystonePosition);
        }
    }

}
