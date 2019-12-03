package org.firstinspires.ftc.teamcode.opmodes.auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;


import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleTankDriveBase;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleTankDriveREVOptimized;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Hook;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "auto",name="Blue Quarry Auto")
public class BlueQuarryAuto extends AlmondLinear {

    boolean isLastStone;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new DriveTrain(hardwareMap);
        hook = new Hook(hardwareMap);

        double offset = 0;

        ElapsedTime timer = new ElapsedTime();
        initNav();

        hook.retract();

        waitForStart();

        if (isStopRequested()) return;


        //Goes forward  to aline with a stone
        forward(21);

        //turns towards quarry
        turn(-90);


        CameraDevice.getInstance().setFlashTorchMode(true);
        timer.reset();
        while (timer.milliseconds() < 1000&&!isStopRequested()) {
            Nav();
        }

        if (skystoneVisible) {
            offset = 1;
            isLastStone = true;
        } else {
            back(8);
            timer.reset();
            while (timer.milliseconds() < 1000&&!isStopRequested()) {
                Nav();
            }
            if (skystoneVisible) {
                offset = 8;
            } else {
                back(8);
                offset = 17;

            }
        }
        CameraDevice.getInstance().setFlashTorchMode(false);

        forward(5);

        driveSideways(0.5, 1000);
        turn(-90);

        hook.extend();
        timer.reset();
        while (timer.milliseconds() < 500 && !isStopRequested()) {
        }

        driveSideways(-0.5, 1300);
        turn(-90);
        //drives into build zone
        back(56 - offset);

        hook.retract();
        timer.reset();
        while (timer.milliseconds() < 500 && !isStopRequested()) {
        }

        if(isLastStone){
            offset = 8;
        }

        //drives back to quarry for second stone
        forward(80 - offset);
        //drives into quarry to pick up stone
        driveSideways(0.5, 1000);
        turn(-90);

        hook.extend();
        timer.reset();
        while (timer.milliseconds() < 500&&!isStopRequested()) {
        }

        //drives away from quarry with stone
        driveSideways(-0.5, 1000);
        turn(-90);

        //drives into build zone again
        back(80 - offset);

        hook.retract();
        timer.reset();
        while (timer.milliseconds() < 500&&!isStopRequested()) {
        }

        forward(8);


        driveSideways(0.5, 1000);
    }

}