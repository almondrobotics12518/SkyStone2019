package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Detector;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.RightGrab;
import org.yaml.snakeyaml.scanner.Constant;

import java.util.Vector;

@Autonomous()
public class BlueAutoGrab extends LinearOpMode {


    private DriveTrain drive;
    private RightGrab rightGrab;
    private Detector detector;

    private int stonePosition = 0;


    public void runOpMode() throws InterruptedException {


        drive = new DriveTrain(hardwareMap);
        detector = new Detector(hardwareMap);
        //rightGrab = new RightGrab(hardwareMap);
        rightGrab = new RightGrab(hardwareMap);

        rightGrab.retract();
        rightGrab.open();

        drive.setPoseEstimate(new Pose2d(-39, 62, Math.toRadians(-90)));
        //rightGrab.open();

        detector.startStreaming();
        while (!isStarted() && !isStopRequested()) {
            double position = detector.detector.foundRectangle().x + detector.detector.foundRectangle().width / 2;
            if (position < 70) {
                stonePosition = 1;
            } else if (position < 125) {
                stonePosition = 2;
            } else {
                stonePosition = 0;
            }
            telemetry.addData("Stone Position: ", stonePosition);
            telemetry.addData("Position: ", position);
            telemetry.update();
        }
        detector.phoneCam.stopStreaming();

        double offset = 0;
        if (stonePosition == 1) {
            offset = 0.5;
        }

        if (stonePosition == 2) {
            offset = 0.5;
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-53,36,Math.toRadians(180)))
                .build());
        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(-44-(stonePosition*8)-offset,33),new ConstantInterpolator(Math.PI))
                .build());
        update();

        rightGrab.setBigPosition(0.95);
        sleep(700);
        rightGrab.close();
        sleep(500);
        rightGrab.setBigPosition(0.55);
        sleep(500);



        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(-12,40,Math.toRadians(180)),new ConstantInterpolator(Math.PI))
                .lineTo(new Vector2d(12,40))
                .splineTo(new Pose2d(49,32.2,Math.toRadians(180)), new ConstantInterpolator(Math.PI))
                .build());

        update();


        rightGrab.setBigPosition(0.75);
        sleep(600);
        rightGrab.open();
        sleep(500);
        rightGrab.retract();
        sleep(500);





        double yOffset = 0;
        if(stonePosition == 1){
            yOffset = 0;
        }
        if(stonePosition == 2){
            yOffset = 0;
        }

        double xOffset = 0;
        if(stonePosition == 2){
            xOffset = 0;
        }
        if(stonePosition == 1){
            xOffset = 0;
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(-5,36),new ConstantInterpolator(Math.PI))
                .splineTo(new Pose2d(-19-stonePosition*8-xOffset,32-yOffset,Math.PI),new ConstantInterpolator(Math.PI))
                .build());
        update();

        rightGrab.extend();
        sleep(700);
        rightGrab.close();
        sleep(500);
        rightGrab.setBigPosition(0.55);
        sleep(500);



        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d( -19-stonePosition*8,38),new ConstantInterpolator(Math.PI))
                .build());
        update();


        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(12,38),new ConstantInterpolator(Math.PI))
                .reverse()
                .splineTo(new Pose2d(58,32,Math.PI),new ConstantInterpolator(Math.PI))
                .build());
        update();

        rightGrab.setBigPosition(0.75);
        sleep(600);
        rightGrab.open();
        sleep(500);
        rightGrab.retract();
        sleep(500);

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(5,34,Math.PI),new ConstantInterpolator(Math.PI))
                .build());
        update();



/*
        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(0,36,Math.toRadians(180)))
                .lineTo(new Vector2d(40,36)).build());

        update();

        rightGrab.extend();
        sleep(700);
        rightGrab.open();
        sleep(250);
        rightGrab.retract();
        sleep(500);

        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(0,36)).build());

        update();
        */

    }
    public void update(){
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
        }
    }


}
