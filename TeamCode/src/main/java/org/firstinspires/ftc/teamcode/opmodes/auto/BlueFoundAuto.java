package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@Autonomous(group="auto",name="Blue Foundation Auto")
public class BlueFoundAuto extends LinearOpMode {

    private DriveTrain drive;


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new DriveTrain(hardwareMap);


        drive.setPoseEstimate(new Pose2d(40,62,Math.toRadians(90)));

        waitForStart();

        // goes to foundation
        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(50,35,Math.toRadians(90)))
                .build());
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
        }

        //

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(40,40,Math.toRadians(180)))
                .build());
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
        }

        drive.followTrajectory(drive.trajectoryBuilder().back(7).build());
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
        }

        drive.followTrajectory(drive.trajectoryBuilder().forward(47).build());
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
        }









    }

}
