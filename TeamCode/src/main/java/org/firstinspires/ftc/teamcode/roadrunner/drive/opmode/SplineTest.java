package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleTankDriveBase;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleTankDriveREVOptimized;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new DriveTrain(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(18, 12, 0))
                        .build()
        );

        sleep(2000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0, 0, 0))
                        .build()
        );
    }
}
