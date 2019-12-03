package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@Autonomous(group="auto",name="Blue Foundation Auto")
public class BlueFoundationAuto extends AlmondLinear {


    public void runOpMode() throws InterruptedException {

        drive = new DriveTrain(hardwareMap);
        claw = new Claw(hardwareMap);


        ElapsedTime totalTime = new ElapsedTime();
        ElapsedTime timer = new ElapsedTime();
        claw.retract();

        waitForStart();

        totalTime.reset();

        drive.followTrajectory(
                drive.trajectoryBuilder().
                        reverse().
                        splineTo(new Pose2d(-30,-10,0)).
                        lineTo(new Vector2d(-35,-10)).
                        build()
        );

        while(drive.isBusy() && isStarted() && !isStopRequested()){
            drive.update();
        }

        back(1);
        claw.extend();
        timer.reset();
        while(timer.milliseconds()<1000 && !isStopRequested() && isStarted()){}
        back(1);
        turn(0);

        forward(33.5);

        turn(30);
        claw.retract();


        timer.reset();
        while(timer.milliseconds()<1000 && !isStopRequested() && isStarted()){}

        turn(0);

        while(totalTime.milliseconds()<23000 && !isStopRequested()){}

        driveSideways(0.5,4000);
    }

}
