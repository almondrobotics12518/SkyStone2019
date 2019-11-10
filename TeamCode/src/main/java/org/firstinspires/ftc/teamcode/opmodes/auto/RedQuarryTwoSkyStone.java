package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Hook;

@Autonomous(name="Red Quarry Two Skystone")
public class RedQuarryTwoSkyStone extends LinearOpMode {

    SampleMecanumDrive drive;
    Hook hook;
    Claw claw;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new DriveTrain(hardwareMap);

        waitForStart();

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                    .splineTo(new Pose2d(24,32, 0))
                    .build()
        );

    }
}
