package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleTankDriveBase;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleTankDriveREVOptimized;
import org.firstinspires.ftc.teamcode.subsystems.Hook;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(name="Red Depot Auto",group="auto")
public class RedSideAuto extends AlmondLinear {


    public double firstDistance = 8;
    public double firstHeading = -90;
    public double secondDistance = 30;
    public double secondHeading = 0;
    public double thirdDistance = 33;

    public Positions position;

    public void runOpMode() throws InterruptedException{

        drive = new SampleTankDriveREVOptimized(hardwareMap);
        hook = new Hook(hardwareMap);

        initVuforia();
        initTF();

        while(!isStarted() && !isStopRequested()){
            position = autoPath();
        }

        waitForStart();

        switch(position){
            case LEFT:
                firstDistance = -8;
                thirdDistance = 49;
                break;
            case MIDDLE:
                firstDistance = 0;
                thirdDistance = 41;
                break;
            case RIGHT:
                firstDistance = 8;
                thirdDistance = 33;
                break;
        }

        if(position != Positions.MIDDLE) {
            drive.followTrajectorySync(drive.trajectoryBuilder()
                    .forward(firstDistance)
                    .build());
        }

        turnTo(Math.toRadians(firstHeading));

        drive.followTrajectory(drive.trajectoryBuilder()
                .forward(secondDistance)
                .build());

        turnTo(0);

        hook.extend();

        drive.setDrivePower(new Pose2d(
                0,
                0.5,
                0
        ));

        waitForTime(300);

        drive.setDrivePower(new Pose2d(
                0,
                0,
                0
        ));


        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(thirdDistance)
                .build());

        hook.retract();

        waitForTime(500);

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(-9)
                .build());


    }

    public void turnTo(double radians) {
        drive.updatePoseEstimate();
        double targetAngle = radians;
        double kp = 0.03;
        double error = targetAngle - drive.getPoseEstimate().getHeading();
        while (!isStopRequested() && Math.abs(error) > 0.01) {
            error = targetAngle - drive.getPoseEstimate().getHeading();

            if (error > Math.PI) {
                error -= Math.PI * 2;
            }

            if (error <= -Math.PI) {
                error += 2 * Math.PI;
            }

            if (Math.abs(error * kp) > 0.5) {
                double power = (0.5 * error * kp) / Math.abs(error * kp);
            }

            drive.setDrivePower(new Pose2d(
                    0,
                    0,
                    error * kp
            ));

            drive.updatePoseEstimate();

        }
    }

    public void waitForTime(double millis){
        ElapsedTime time = new ElapsedTime();
        while(!isStopRequested() && time.milliseconds()<millis){

        }
    }


}
