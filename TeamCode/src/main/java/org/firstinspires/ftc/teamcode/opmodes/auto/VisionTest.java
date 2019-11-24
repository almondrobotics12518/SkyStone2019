package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;



import java.util.List;

@TeleOp(group="test",name="Vision Test")
public class VisionTest extends AlmondLinear {

    public Positions skystonePosition;

    public void runOpMode(){

        initNav();


        ElapsedTime timer = new ElapsedTime();


        waitForStart();

        while(opModeIsActive()){
            Nav();

        }


    }



}
