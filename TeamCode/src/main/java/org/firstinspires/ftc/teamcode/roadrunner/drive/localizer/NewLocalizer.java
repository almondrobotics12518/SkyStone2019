package org.firstinspires.ftc.teamcode.roadrunner.drive.localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import android.support.annotation.NonNull;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxOptimizedI2cFactory;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.TRACK_WIDTH;

public class NewLocalizer implements Localizer {

    private ExpansionHubEx hub;
    private BNO055IMU imu;

    private ExpansionHubMotor leftFront, leftRear, rightFront, rightRear;

    private List<Double> lastWheelPositions = Arrays.asList(0.0,0.0);
    private List<Double> currentWheelPositions = Arrays.asList(0.0,0.0);
    private double lastHeading=0, currentHeading=0;
    private double deltaX, deltaY;
    private Pose2d poseEstimate = new Pose2d(0,0,0),lastPoseEstimate=new Pose2d(0,0,0);

    public NewLocalizer(HardwareMap hardwareMap){

        hub = hardwareMap.get(ExpansionHubEx.class, "hub");


        //Gyro Calibration

        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        while(!imu.isGyroCalibrated()){ }

        //Motor mapping
        leftFront = hardwareMap.get(ExpansionHubMotor.class, "leftFront");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "leftRear");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "rightRear");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "rightFront");
    }

    public double getHeading(){
        return imu.getAngularOrientation().firstAngle;
    }

    public List<Double> getWheelPositions(){
        double rightDistance = 0, leftDistance = 0;
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null){ return Arrays.asList(0.0,0.0); }

        rightDistance = DriveConstants.encoderTicksToInches(bulkData.getMotorCurrentPosition(rightFront)+bulkData.getMotorCurrentPosition(rightRear))/2;
        leftDistance = DriveConstants.encoderTicksToInches(bulkData.getMotorCurrentPosition(leftFront)+bulkData.getMotorCurrentPosition(leftRear)/2);
        return Arrays.asList(
                rightDistance,
                leftDistance
        );

    }

    public void update(){
        if(imu!=null) {
            currentHeading = getHeading();
        } else { currentHeading = lastHeading; }

        currentWheelPositions = getWheelPositions();
        double distance = (currentWheelPositions.get(0)-lastWheelPositions.get(0)+currentWheelPositions.get(1)-lastWheelPositions.get(1))/2;

        deltaX = Math.cos(lastHeading)*distance;
        deltaY = Math.sin(lastHeading)*distance;

        poseEstimate = new Pose2d(
                lastPoseEstimate.getX()+deltaX,
                lastPoseEstimate.getY()+deltaY,
                currentHeading
        );
        lastWheelPositions = currentWheelPositions;
        lastHeading = currentHeading;
        lastPoseEstimate = poseEstimate;
    }

    public void setPoseEstimate(Pose2d estimate){ poseEstimate = estimate; }

    public Pose2d getPoseEstimate(){ return poseEstimate; }
}