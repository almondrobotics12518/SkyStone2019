package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;
import org.firstinspires.ftc.teamcode.subsystems.Detector;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LiftExt;

@Autonomous(group="auto",name="Blue Auto")
public class BlueAuto extends AlmondLinear {

    private LiftExt lift;
    private DriveTrain drive;
    private Detector detector;
    private Intake intake;

    private double stonePosition = 0;


    public void runOpMode() throws InterruptedException {

        lift = new LiftExt(this);
        drive = new DriveTrain(hardwareMap);
        detector = new Detector(hardwareMap);
        intake = new Intake(this);

        detector.startStreaming();
        detector.phoneCam.pauseViewport();
        while(!isStarted()&&!isStopRequested()){
            double position = detector.detector.foundRectangle().x+detector.detector.foundRectangle().width/2;
            if(position<105){
                stonePosition = 1;
            } else if(position<175){
                stonePosition = 2;
            } else {
                stonePosition = 0;
            }
            telemetry.addData("Stone Position: ",stonePosition);
            telemetry.update();
        }
        detector.phoneCam.stopStreaming();

    }

}
