package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Detector;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LiftExt;

import kotlin.Unit;

@Autonomous(group="auto",name="Blue Auto")
public class BlueAuto extends LinearOpMode {

    private LiftExt lift;
    private DriveTrain drive;
    private Detector detector;
    private Intake intake;
    private Claw claw;

    private int stonePosition = 0;


    public void runOpMode() throws InterruptedException {

        lift = new LiftExt(hardwareMap);
        drive = new DriveTrain(hardwareMap);
        detector = new Detector(hardwareMap);
        intake = new Intake(this);
        claw = new Claw(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-39,62,Math.toRadians(-90)));
        claw.open();

        detector.startStreaming();
        while(!isStarted()&&!isStopRequested()){
            double position = detector.detector.foundRectangle().x+detector.detector.foundRectangle().width/2;
            if(position<70){
                stonePosition = 1;
            } else if(position<125){
                stonePosition = 2;
            } else {
                stonePosition = 0;
            }
            telemetry.addData("Stone Position: ",stonePosition);
            telemetry.addData("Position: ",position);
            telemetry.update();
        }
        detector.phoneCam.stopStreaming();

        drive.followTrajectory(drive.trajectoryBuilder()
                .addMarker(()->{intake.intake(); return Unit.INSTANCE;})
                .splineTo(new  Pose2d(-39-(stonePosition*8),38,Math.toRadians(-135)))
                .lineTo(new Vector2d(-44-(stonePosition*8),20),new ConstantInterpolator(Math.toRadians(-135)))
                .build());
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
            lift.update();
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .addMarker(1,()->{claw.close(); return Unit.INSTANCE;})
                .splineTo(new Pose2d(0,36,Math.toRadians(-180)))//Moves to go under skybridge
                .splineTo(new Pose2d(48,32,Math.toRadians(-270)))//Goes to the foundation);
                .build());
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
            lift.update();
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
        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(0,36,Math.toRadians(-180)))//Goes back to under the skybridge
                .splineTo(new  Pose2d(-18-(stonePosition*8),32,Math.toRadians(-130)))
                .lineTo(new Vector2d(-22-(stonePosition*8),20),new ConstantInterpolator(Math.toRadians(-130)))
                .build());
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
            lift.update();
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(0,34,Math.toRadians(-180)))
                .lineTo(new Vector2d(36,34))
                .build());
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
            lift.update();
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(0,36))
                .build());
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
            lift.update();
        }


    }

    public void update(){
        drive.update();
        lift.update();
    }

}
