package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LiftExt;

@Config
@TeleOp()
public class LiftUpTest extends LinearOpMode {

    private  FtcDashboard dashboard;

    public static double DISTANCE = 20;

    public void runOpMode() throws InterruptedException {
        LiftExt lift = new LiftExt(this);
        dashboard = FtcDashboard.getInstance();

        waitForStart();

        lift.setHeight(DISTANCE);
        while(!isStopRequested()){
            lift.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Position",lift.getCurrentHeight());
            packet.put("Error",lift.getDesiredMotionState().getX()-lift.getCurrentHeight());
            dashboard.sendTelemetryPacket(packet);
        }

    }

}
