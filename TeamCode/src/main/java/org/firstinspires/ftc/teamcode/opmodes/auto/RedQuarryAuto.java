package org.firstinspires.ftc.teamcode.opmodes.auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Claw;


import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleTankDriveBase;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleTankDriveREVOptimized;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Hook;


@Autonomous(name="Red Quarry Auto",group="auto")
public class RedQuarryAuto extends AlmondLinear {



    public void runOpMode() throws InterruptedException {
        //Init code
        telemetry.addLine("Initializing...");
        telemetry.update();

        drive = new DriveTrain(hardwareMap);
        hook = new Hook(hardwareMap);
        claw = new Claw(hardwareMap);


        telemetry.addLine("Initialized");
        telemetry.update();
        //servo initialization
        hook.retract();



        waitForStart();

        //positive sideways left
        //negative sideways right

        if (isStopRequested()) return;



        //Goes forward  to aline with a stone
        forward(21);

        //turns towards quarry
        turn(-90);


        /*
        if(targetVisible){
            hook.extends();
        }else {
            forward(8);
            if(targetVisible){

            }else{
                forward(8);
            }
        }

         */





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
}
