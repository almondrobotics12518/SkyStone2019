package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Hook;

@Autonomous(name="Blue Quarry Auto",group="auto")
public class BlueQuarryAuto extends LinearOpMode {
    SampleMecanumDrive drive;
    Hook hook;
    Claw claw;
    ElapsedTime timer;

    public void runOpMode() throws InterruptedException {
        //Init code
        telemetry.addLine("Initializing...");
        telemetry.update();

        drive = new DriveTrain(hardwareMap);
        hook = new Hook(hardwareMap);
        claw = new Claw(hardwareMap);
        timer = new ElapsedTime();

        telemetry.addLine("Initialized");
        telemetry.update();
        //servo initialization
        hook.retract();

        waitForStart();

        drive.turn(Math.toRadians(-90));
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
        .splineTo(new Pose2d(24,32,Math.PI))
        .build());

        while(!isStopRequested() && drive.isBusy()){drive.update();}

        hook.extend();




    }
}
