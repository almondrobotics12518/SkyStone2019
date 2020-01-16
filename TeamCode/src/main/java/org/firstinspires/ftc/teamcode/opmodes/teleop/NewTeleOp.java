package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Hook;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LiftExt;



@TeleOp(name="New TeleOp",group="teleop")
public class NewTeleOp extends LinearOpMode {

    private Claw claw;
    private DriveTrain dt;
    private Hook hook;
    private Intake intake;
    private LiftExt lift;

    private boolean hookIsClosed = false;
    private boolean clawIsClosed = false;
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;


    @Override
    public void runOpMode() throws InterruptedException {

        claw = new Claw(hardwareMap);
        hook = new Hook(hardwareMap);
        dt = new DriveTrain(hardwareMap);
        intake = new Intake(this);
        lift = new LiftExt(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            // drivetrain
            dt.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * 0.5));

            // slide
            lift.setSlidePower(-gamepad2.right_stick_y);

            // lift
            lift.setPower(-gamepad2.left_stick_y);

            // intake
            intake.setPower(gamepad2.right_trigger-gamepad2.left_trigger);

            // hook

            if(gamepad1.a && !aWasPressed){
                aWasPressed = true;
                hook.toggle();
            }
            if(!gamepad1.a){
                aWasPressed = false;
            }

            // claw

            if(gamepad2.b && !bWasPressed){
                bWasPressed = true;
                claw.toggle();
            }
            if(!gamepad2.b){
                bWasPressed = false;
            }

            telemetry.addData("Wheels",dt.getWheelPositions());
            //telemetry.addData("Lift",lift.getCurrentHeight());
            telemetry.addData("Gamepad2 stick: ",gamepad2.left_stick_y);
            telemetry.update();


        }
    }
}
