package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@Autonomous(group="auto",name="Auto Test")
public class AutoTest extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Claw claw;

    public void runOpMode() throws InterruptedException {
        drive = new DriveTrain(hardwareMap);
        claw = new Claw(hardwareMap);

        ElapsedTime timer = new ElapsedTime();
        claw.retract();
        waitForStart();

        drive.followTrajectory(drive.trajectoryBuilder()
                .back(33)
                .build());
        while(!isStopRequested()&&isStarted()){
            drive.update();
        }

        claw.extend();
        timer.reset();
        while(!isStopRequested()&&timer.milliseconds()<1000){
            drive.update();
        }
        drive.followTrajectory(drive.trajectoryBuilder()
            .forward(33).build());

        while(!isStopRequested()&&isStarted()){
            drive.update();
        }

    }

}
