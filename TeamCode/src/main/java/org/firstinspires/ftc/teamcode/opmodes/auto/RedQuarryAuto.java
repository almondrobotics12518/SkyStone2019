package org.firstinspires.ftc.teamcode.opmodes.auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleTankDriveBase;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleTankDriveREVOptimized;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Hook;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "auto",name="Red Quarry Auto")
public class RedQuarryAuto extends AlmondLinear {

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


        timer.reset();
        while (timer.milliseconds() < 1000&&!isStopRequested()) {
            Nav();
        }

        if (targetVisible) {
            offset = 0;
        } else {
            back(8);
            timer.reset();
            while (timer.milliseconds() < 1000&&!isStopRequested()) {
                Nav();
            }
            if (targetVisible) {
                offset = 8;
            } else {
                back(7);
                offset = 15;
                isLastStone = true;
            }
        }

        forward(4);

        intake();
        //drives into build zone
        forward(48 + offset);

        hook.retract();
        timer.reset();
        while (timer.milliseconds() < 500 && !isStopRequested()) {
        }

        if(isLastStone){
            offset = 8;
        }
        //drives back to quarry for second stone
        back(71 + offset);
        //drives into quarry to pick up stone
        intake();

        //drives into build zone again
        forward(71 + offset - 8);

        hook.retract();
        timer.reset();
        while (timer.milliseconds() < 500&&!isStopRequested()) {
        }

        back(8);

    }

}