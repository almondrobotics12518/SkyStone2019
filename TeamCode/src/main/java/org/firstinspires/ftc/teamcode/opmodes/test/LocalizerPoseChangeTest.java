package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.LiftExt;

@TeleOp()
public class LocalizerPoseChangeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain drive = new DriveTrain(hardwareMap);

        drive.setPoseEstimate(new Pose2d(10,10,Math.PI/2));

        waitForStart();

        while(!isStopRequested()){
            drive.updatePoseEstimate();
            telemetry.addData("Pose estimate: ",drive.getPoseEstimate());
            telemetry.update();
        }

    }
}
