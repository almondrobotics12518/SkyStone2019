package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;
import org.firstinspires.ftc.teamcode.subsystems.Detector;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftExt;

@Autonomous(group="auto",name="Red Auto")
public class RedAuto extends LinearOpMode {


    private LiftExt lift;
    private Detector detector;
    private DriveTrain drive;
    private Intake intake;

    private int stonePosition;

    @Override
    public void runOpMode() throws InterruptedException {

        lift = new LiftExt(hardwareMap);
        drive = new DriveTrain(hardwareMap);
        detector = new Detector(hardwareMap);
        intake = new Intake(this);

        drive.setPoseEstimate(new Pose2d(-33,-63,Math.toRadians(90)));

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

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-48-(stonePosition*8),-24,(Math.toRadians(135))))//Moves to go pick up 1st skystone
                .reverse()
                .splineTo(new Pose2d(0,-36,Math.toRadians(-180)))//Moves to go under skybridge
                .splineTo(new Pose2d(48,-33,Math.toRadians(270)))//Goes to the foundation
                .reverse()
                .splineTo(new Pose2d(40,-40,Math.toRadians(-180)))//Moves backwards and turns
                .reverse()
                .splineTo(new Pose2d(48,-48,Math.toRadians(-180)))//Goes  to drop off spot of foundation
                .reverse()
                .splineTo(new Pose2d(0,-36,Math.toRadians(-180)))//Goes back to under the skybridge
                .splineTo(new Pose2d(-24-(stonePosition*8),-24,Math.toRadians(135)))//Goes back to pick up 2nd skystone
                .reverse()
                .splineTo(new Pose2d(0,-36,Math.toRadians(-180)))
                .lineTo(new Vector2d(36,-36))
                .reverse()
                .lineTo(new Vector2d(0,-36))
                .build());
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
            lift.update();
        }



    }

}


