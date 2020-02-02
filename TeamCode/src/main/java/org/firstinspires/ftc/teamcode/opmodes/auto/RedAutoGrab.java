package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Detector;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.LeftGrab;

@Autonomous()
public class RedAutoGrab extends LinearOpMode {

    int stonePosition;

    private DriveTrain drive;
    private Detector detector;
    private LeftGrab leftGrab;

    public void runOpMode() throws InterruptedException {

        drive = new DriveTrain(hardwareMap);
        detector = new Detector(hardwareMap);
        leftGrab = new LeftGrab(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-39, -62, Math.toRadians(90)));
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

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-10,-36,0))
                .build());
        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(-20-(8*stonePosition),-32,0),new ConstantInterpolator(0))
                .build());
        update();

        leftGrab.extend();
        sleep(500);
        leftGrab.close();
        sleep(500);
        leftGrab.setBigPosition(0.4);
        sleep(400);

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-10,-40,0),new ConstantInterpolator(0))
                .lineTo(new Vector2d(10,-40),new ConstantInterpolator(0))
                .splineTo(new Pose2d(49,-31,0),new ConstantInterpolator(0))
                .build());
        update();

        leftGrab.extend();
        sleep(500);
        leftGrab.open();
        sleep(500);
        leftGrab.retract();
        sleep(400);

        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(10,-40,0),new ConstantInterpolator(0))
                .lineTo(new Vector2d(-10,-40), new ConstantInterpolator(0))
                .splineTo(new Pose2d(-44-(8*stonePosition)),new ConstantInterpolator(0))
                .build());

        while(!isStopRequested()){

        }

    }

    public void update(){
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
        }
    }


}
