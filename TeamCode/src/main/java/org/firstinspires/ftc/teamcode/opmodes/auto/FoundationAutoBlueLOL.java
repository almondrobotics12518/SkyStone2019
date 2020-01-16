package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.os.Build;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Hook;

import java.util.Vector;

@Autonomous
public class FoundationAutoBlueLOL extends LinearOpMode {

    private DriveTrain drive;
    private Hook hook;

    public void runOpMode() throws InterruptedException{

        drive  = new DriveTrain(hardwareMap);
        hook = new Hook(hardwareMap);

        drive.setPoseEstimate(new Pose2d(40,62,Math.toRadians(90)));
        waitForStart();

        drive.followTrajectory(drive.trajectoryBuilder()
        .back(1)
                .reverse()
        .splineTo(new Pose2d(50,35,Math.toRadians(90)))
        .lineTo(new Vector2d(50,30),new ConstantInterpolator(Math.toRadians(90)))
                .build());

        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
        }

        hook.close();
        sleep(500);

        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(40,48),new ConstantInterpolator(Math.toRadians(90)))
                .build());
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
        }

        drive.turn(Math.toRadians(90));
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
        }

        drive.followTrajectory(drive.trajectoryBuilder()
        .lineTo(new Vector2d(55,46),new ConstantInterpolator(Math.toRadians(180)))
        .build());

        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
        }

        hook.open();
        sleep(300);


        drive.followTrajectory(drive.trajectoryBuilder().forward(47).build());
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
        }


    }
}
