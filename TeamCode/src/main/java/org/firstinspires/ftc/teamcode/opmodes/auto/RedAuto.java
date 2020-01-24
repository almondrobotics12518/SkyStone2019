package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;
import org.firstinspires.ftc.teamcode.subsystems.Detector;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftExt;

import kotlin.Unit;

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
            if(position<40){
                stonePosition = 2;
            } else if(position<100){
                stonePosition = 1;
            } else {
                stonePosition = 0;
            }
            telemetry.addData("Stone Position: ",stonePosition);
            telemetry.update();
        }
        detector.phoneCam.stopStreaming();

        double  offset = 0;
        if(stonePosition ==1){
            offset = 2;
        }

        if(stonePosition==2){
            offset = 2.5;
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                .addMarker(()->{intake.setPower(0.5); return Unit.INSTANCE;})
                .splineTo(new  Pose2d(-40-(stonePosition*8)+offset,-38,Math.toRadians(135)))
                .lineTo(new Vector2d(-44-(stonePosition*8)+offset,-20),new ConstantInterpolator(Math.toRadians(135)))
                .build());
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
            lift.update();
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .addMarker(0.5,()->{intake.setPower(0.2); return Unit.INSTANCE;})
                .splineTo(new Pose2d(0,-34,Math.toRadians(-180)))//Moves to go under skybridge
                //.addMarker(()->{lift.setHeight(9); lift.setSlidePower(-1); return Unit.INSTANCE;})
                .splineTo(new Pose2d(24,-34.1,Math.toRadians(180)))//Goes to the foundation);

                //.addMarker(()->{lift.setSlidePower(0); claw.open(); return Unit.INSTANCE;})
                .build());
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
            lift.update();
        }

        drive.turn(Math.toRadians(-90));
        while(drive.isBusy()&&!isStopRequested()){
            drive.update();
        }
        intake.setPower(-1);
        drive.turn(Math.toRadians(90));
        while(drive.isBusy()&&!isStopRequested()){
            drive.update();
        }
/*
        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(48,48,Math.toRadians(-180)))//Goes  to drop off spot of foundation
                .build());
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
            lift.update();
        }
*/

        double offset2 = 0;
        if(stonePosition == 1){
            offset2 = 2.5;
        }
        if(stonePosition == 2){
            offset2 = 2.5;
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                //.addMarker(()->{lift.setHeight(0.1); return Unit.INSTANCE;})

                .splineTo(new Pose2d(0,-34,Math.toRadians(-180)))//Goes back to under the skybridge
                .addMarker(()->{intake.setPower(0.5); return Unit.INSTANCE;})
                .splineTo(new  Pose2d(-18-(stonePosition*8)+offset2,-32,Math.toRadians(130)))
                .lineTo(new Vector2d(-24-(stonePosition*8)+offset2,-20),new ConstantInterpolator(Math.toRadians(130)))
                .build());
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
            lift.update();
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .addMarker(0.5,()->{intake.setPower(0.2); return Unit.INSTANCE;})
                .splineTo(new Pose2d(0,-34,Math.toRadians(-180)))
                .splineTo(new Pose2d(20,-34.1,Math.toRadians((180))))
                .build());
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
            lift.update();
        }

        drive.turn(Math.toRadians(-90));
        while(drive.isBusy()&&!isStopRequested()){
            drive.update();
        }
        intake.setPower(-1);
        drive.turn(Math.toRadians(90));
        while(drive.isBusy()&&!isStopRequested()){
            drive.update();
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(0,-34,Math.PI))
                .build());
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
            lift.update();
        }


    }

}


