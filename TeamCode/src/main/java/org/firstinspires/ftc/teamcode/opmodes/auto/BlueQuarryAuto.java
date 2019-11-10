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
public class BlueQuarryAuto extends AlmondLinear {
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
        back(7);

        //turns towards quarry
        turn(90);

        //moves forward towards quarry
        forward(26);

        //turns to intake
        turn(0);

        //drives away from quarry
        driveSideways(-0.5,1000);
        turn(0);

        //drives into build zone
        back(48);

        //drives back to quarry for second stone
        forward(78);
        //drives into quarry to pick up stone
        driveSideways(0.5,1200);
        turn(0);

        //drives away from quarry with stone
        driveSideways(-0.5,1300);
        turn(0);

        //drives into build zone again
        back(78);



    }




}
