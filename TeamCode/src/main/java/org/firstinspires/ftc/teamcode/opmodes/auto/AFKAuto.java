package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@Autonomous()
public class AFKAuto extends LinearOpMode {

    private DriveTrain drive;

    @Override
    public void runOpMode(){
        drive = new DriveTrain(hardwareMap);

        waitForStart();

        drive.followTrajectory(drive.trajectoryBuilder()
            .forward(24)
            .build());

        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
        }
    }

}
