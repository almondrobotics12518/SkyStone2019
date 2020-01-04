package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.LiftExt;

@Config
@TeleOp()
public class LiftTest extends LinearOpMode {

    private FtcDashboard dashboard;

    public static double HEIGHT = 20;

    public void runOpMode() throws InterruptedException{
        dashboard = FtcDashboard.getInstance();

        ElapsedTime timer = new ElapsedTime();


        LiftExt lift = new LiftExt(this);
        waitForStart();
        while(!isStopRequested()){
            lift.setHeight(5 + Math.random()*25);
            while(!isStopRequested()&&lift.isBusy()){
                lift.update();
                double currentHeight = lift.getCurrentHeight();
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Position",currentHeight);
                packet.put("Error",lift.getDesiredMotionState().getX()-currentHeight);
                dashboard.sendTelemetryPacket(packet);
            }
            timer.reset();
            while(!isStopRequested()&&timer.milliseconds()<1000){
                lift.update();
                double currentHeight = lift.getCurrentHeight();
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Position",currentHeight);
                packet.put("Error",lift.getDesiredMotionState().getX()-currentHeight);
                dashboard.sendTelemetryPacket(packet);
            }



        }
    }

}
