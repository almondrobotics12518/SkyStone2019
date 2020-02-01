package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Detector;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@Autonomous(group="auto",name="Blue Auto")
public class BlueAutoGrab extends LinearOpMode {


    private DriveTrain drive;
    //private RightGrab rightGrab;
    private Detector detector;

    private int stonePosition = 0;


    public void runOpMode() throws InterruptedException {


        drive = new DriveTrain(hardwareMap);
        detector = new Detector(hardwareMap);
        //rightGrab = new RightGrab(hardwareMap);


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
            offset = 2;
        }

        if (stonePosition == 2) {
            offset = 1.5;
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new  Pose2d(-44-(stonePosition*8)+offset,34,Math.toRadians(180)))
                .build());

        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(0,36,Math.toRadians(180)))
                .lineTo(new Vector2d(40,36)).build());

        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(0,36))
                .splineTo(new  Pose2d(-20-(stonePosition*8)+offset,34,Math.toRadians(180)))
                .build());

        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(0,36,Math.toRadians(180)))
                .lineTo(new Vector2d(40,36)).build());

        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(0,36)).build());

        update();

    }
    public void update(){
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
        }
    }
}
