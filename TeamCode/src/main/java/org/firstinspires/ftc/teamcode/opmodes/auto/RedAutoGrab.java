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
import org.firstinspires.ftc.teamcode.subsystems.RightGrab;

@Autonomous()
public class RedAutoGrab extends LinearOpMode {

    int stonePosition;

    private RightGrab rightGrab;
    private DriveTrain drive;
    private Detector detector;
    private LeftGrab leftGrab;

    public void runOpMode() throws InterruptedException {

        rightGrab = new RightGrab(hardwareMap);
        drive = new DriveTrain(hardwareMap);
        detector = new Detector(hardwareMap);
        leftGrab = new LeftGrab(hardwareMap);

        rightGrab.retract();
        rightGrab.open();

        leftGrab.retract();
        leftGrab.open();

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
                .splineTo(new Pose2d(-20-(8*stonePosition),-31.5,0),new ConstantInterpolator(0))
                .build());
        update();

        leftGrab.extend();
        sleep(400);
        leftGrab.close();
        sleep(400);
        leftGrab.setBigPosition(0.4);
        sleep(300);

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-10,-38,0),new ConstantInterpolator(0))
                .lineTo(new Vector2d(10,-38),new ConstantInterpolator(0))
                .splineTo(new Pose2d(49,-28.5,0),new ConstantInterpolator(0))
                .build());
        update();

        leftGrab.extend();
        sleep(400);
        leftGrab.open();
        sleep(400);
        leftGrab.retract();
        sleep(300);

        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(10,-38,0),new ConstantInterpolator(0))
                .lineTo(new Vector2d(-10,-38), new ConstantInterpolator(0))
                .splineTo(new Pose2d(-44-(8*stonePosition),-30.5),new ConstantInterpolator(0))
                .build());
        update();

        leftGrab.extend();
        sleep(400);
        leftGrab.close();
        sleep(400);
        leftGrab.setBigPosition(0.4);
        sleep(300);

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-10,-38,0),new ConstantInterpolator(0))
                .lineTo(new Vector2d(10,-38),new ConstantInterpolator(0))
                .splineTo(new Pose2d(58,-28.5,0),new ConstantInterpolator(0))
                .build());
        update();

        leftGrab.extend();
        sleep(400);
        leftGrab.open();
        sleep(400);
        leftGrab.retract();
        sleep(300);

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(12,-34.5,0),new ConstantInterpolator(0))
                .lineTo(new Vector2d(6,-34.5), new ConstantInterpolator(0))
                .build());
        update();

    }

    public void update(){
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
        }
    }


}
