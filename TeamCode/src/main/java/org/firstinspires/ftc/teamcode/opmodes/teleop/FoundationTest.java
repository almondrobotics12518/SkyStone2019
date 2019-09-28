package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleTankDriveREVOptimized;

@TeleOp(name="Foundation Move Test",group = "test")
public class FoundationTest extends AlmondLinear {


    @Override
    public void runOpMode() throws InterruptedException{

        waitForStart();
        drive = new SampleTankDriveREVOptimized(hardwareMap);
        drive.followTrajectorySync(drive.trajectoryBuilder()
        .forward(16)
        .build());
    }
}
