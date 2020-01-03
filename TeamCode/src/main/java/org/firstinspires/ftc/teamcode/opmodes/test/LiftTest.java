package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LiftExt;

@Config
@TeleOp()
public class LiftTest extends LinearOpMode {

    private FtcDashboard dashboard;

    public static double HEIGHT = 20;

    public void runOpMode() throws InterruptedException{
        dashboard = FtcDashboard.getInstance();



        LiftExt lift = new LiftExt(this);
        waitForStart();
        lift.setHeight(HEIGHT);
        while(!isStopRequested()){
            lift.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Position",lift.getCurrentHeight());
            dashboard.sendTelemetryPacket(packet);

        }
    }

}
