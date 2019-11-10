package org.firstinspires.ftc.teamcode.opmodes.auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleTankDriveBase;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleTankDriveREVOptimized;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Hook;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "auto",name="Blue Quarry Auto")
public class BlueQuarryAuto extends LinearOpMode {
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new DriveTrain(hardwareMap);
        Hook hook = new Hook(hardwareMap);

        ElapsedTime timer = new ElapsedTime();

        hook.retract();

        waitForStart();

        if (isStopRequested()) return;


        //Goes forward  to aline with a stone
        forward(21);

        //turns towards quarry
        turn(-90);

        //drives away from quarry
        driveSideways(0.5,1000);
        turn(-90);

        //intake here

        driveSideways(-0.5,1000);
        turn(-90);
        //drives into build zone
        back(48);

        //drives back to quarry for second stone
        forward(78);
        //drives into quarry to pick up stone
        driveSideways(0.5,1200);
        turn(-90);

        //drives away from quarry with stone
        driveSideways(-0.5,1300);
        turn(-90);

        //drives into build zone again
        back(78);



    }

    /**
     * this methos is used to turn to an absolute angle
     * @param degrees to turn in an absolute angle
     */
    public void turn(double degrees){
        drive.turn(Angle.normDelta(Math.toRadians(degrees)-drive.getPoseEstimate().getHeading()));
        while(drive.isBusy()&&isStarted()&&!isStopRequested()){
            drive.update();
        }
    }


    /**
     * This method is used to go forwards a speicied number of inches
     * @param inches to travel
     */
    public void forward(double inches){
        drive.followTrajectory(
                drive.trajectoryBuilder().
                        forward(inches).
                        build()
        );
        while(drive.isBusy()&&!isStopRequested()&&isStarted()){
            drive.update();
        }
    }


    /**
     *
     * @param inches to go backwards
     */
    public void back(double inches){
        drive.followTrajectory(
                drive.trajectoryBuilder().
                        back(inches).
                        build()
        );
        while(drive.isBusy()&&!isStopRequested()&&isStarted()){
            drive.update();
        }
    }

    public void driveSideways(double power, int millis){
        ElapsedTime t = new ElapsedTime();
        t.reset();
        drive.setDrivePower(
                new Pose2d(0,power,0)
        );

        while(!isStopRequested() && isStarted() && t.milliseconds()<millis){
            drive.updatePoseEstimate();
        }

        drive.setDrivePower(new Pose2d());
    }


}
