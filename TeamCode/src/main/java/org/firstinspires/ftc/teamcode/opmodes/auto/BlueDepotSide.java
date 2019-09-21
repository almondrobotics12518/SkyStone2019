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

@Autonomous(name="test",group="test")
public class BlueDepotSide extends AlmondLinear {

    public SampleTankDriveBase drive;


    public void runOpMode() throws InterruptedException{

        double LEFT_FIRST_DISTANCE = 8;

        drive = new SampleTankDriveREVOptimized(hardwareMap);

        waitForStart();

        Trajectory second = drive.trajectoryBuilder()
                .forward(30)
                .build();

        drive.followTrajectorySync(drive.trajectoryBuilder()
        .forward(LEFT_FIRST_DISTANCE)
        .build());

        turnTo(Math.PI/2);

        drive.followTrajectory(second);

        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(33)
                .build());



    }

    public void waitForTime(double millis){
        ElapsedTime time = new ElapsedTime();
        while(!isStopRequested() && time.milliseconds()<millis){

        }
    }

    public void turnTo(double radians){
        drive.updatePoseEstimate();
        double targetAngle = radians;
        double kp = 0.025;
        double error = targetAngle - drive.getPoseEstimate().getHeading();
        while(!isStopRequested()&&Math.abs(error)>0.01){
            error = targetAngle - drive.getPoseEstimate().getHeading();

            if(error>Math.PI){
                error -= Math.PI*2;
            }

            if(error<= -Math.PI){
                error += 2 * Math.PI;
            }

            drive.setDrivePower(new Pose2d(
                    0,
                    0,
                    error*kp
            ));

            drive.updatePoseEstimate();

        }

    }
}
