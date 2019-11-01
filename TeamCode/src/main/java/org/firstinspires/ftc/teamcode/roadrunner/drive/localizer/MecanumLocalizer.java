package org.firstinspires.ftc.teamcode.roadrunner.drive.localizer;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxOptimizedI2cFactory;
import org.openftc.revextensions2.ExpansionHubEx;

import java.util.Arrays;
import java.util.List;

public class MecanumLocalizer {
    private MecanumDrive drive;

    private ExpansionHubEx hub;
    private BNO055IMU imu;
    private Pose2d poseEstimate;
    private Pose2d lastPoseEstimate;
    private List<Double> wheelPositions;
    private List<Double> lastWheelPositions;
    private double heading;
    private double lastHeading;

    public MecanumLocalizer(MecanumDrive drive, HardwareMap hardwareMap){
        this.drive = drive;
        poseEstimate = new Pose2d();
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        lastPoseEstimate = new Pose2d();

        while(!imu.isGyroCalibrated()){ }
    }

    public void update(){
        wheelPositions = drive.getWheelPositions();
        heading = getHeading();

        if(lastWheelPositions != null){
            List<Double> wheelDeltas = Arrays.asList(wheelPositions.get(0)-lastWheelPositions.get(0),
                    wheelPositions.get(1)-lastWheelPositions.get(1),
                    wheelPositions.get(2)-lastWheelPositions.get(2),
                    wheelPositions.get(3)-lastWheelPositions.get(3));

            double robotDeltaX = 0;
            for(double position : wheelDeltas) {
                robotDeltaX += position / 4;
            }

            double robotDeltaY = wheelDeltas.get(0) - wheelDeltas.get(1) +wheelDeltas.get(2) - wheelDeltas.get(3);

            double globalDeltaX = Math.cos(heading)*robotDeltaX - Math.sin(heading)*robotDeltaY;
            double globalDeltaY = Math.sin(heading)*robotDeltaX + Math.cos(heading)*robotDeltaY;

            poseEstimate = new Pose2d(
                    poseEstimate.getX() + globalDeltaX,
                    poseEstimate.getY() + globalDeltaY,
                    heading
            );


        }

        lastPoseEstimate = poseEstimate;
        lastWheelPositions = wheelPositions;
        lastHeading = heading;
    }

    public double getHeading(){
        return (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + (2 * Math.PI) )%(2*Math.PI);
    }

    public Pose2d getPoseEstimate(){return poseEstimate;}
}
