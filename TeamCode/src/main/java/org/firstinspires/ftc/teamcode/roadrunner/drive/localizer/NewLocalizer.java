package org.firstinspires.ftc.teamcode.roadrunner.drive.localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import android.support.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxOptimizedI2cFactory;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.encoderTicksToInches;


/*
 * This class holds the localization code based on motor encoders.
 *
 */
public class NewLocalizer implements Localizer {

    private ExpansionHubEx hub;
    private BNO055IMU imu;

    private ExpansionHubMotor leftFront, leftRear, rightFront, rightRear;

    private List<Double> lastWheelPositions;
    private List<Double> currentWheelPositions;
    private double lastHeading=0, currentHeading=0;
    private double headingOffset=0;
    private double deltaX, deltaY;
    private Pose2d poseEstimate = new Pose2d(0,0,0),lastPoseEstimate=new Pose2d(0,0,0);


    /*
     * This constructor initializes everything (the imu calibration code is blocking and shouldn't be there)
     *
     */
    public NewLocalizer(HardwareMap hardwareMap){

        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");


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

        lastWheelPositions = getWheelPositions();
    }

    public void update(){
        //makes sure that imu is initialized
        if(imu!=null) {
            currentHeading = getHeading();
        } else { currentHeading = lastHeading; }

        //Sets the wheel positions to a variable then finds the change in wheel positions
        currentWheelPositions = getWheelPositions();
        List<Double> wheelDeltas = Arrays.asList(currentWheelPositions.get(0)-lastWheelPositions.get(0),
                currentWheelPositions.get(1)-lastWheelPositions.get(1),
                currentWheelPositions.get(2)-lastWheelPositions.get(2),
                currentWheelPositions.get(3)-lastWheelPositions.get(3));

        double robotDeltaX = 0;
        for(double position : wheelDeltas) {
            robotDeltaX += position / 4;
        }

        //currently not using y delta because of drift caused by mecanum drive
        double robotDeltaY = 0;

        //Rotates the relative robot position change to global position change

        deltaX = Math.cos(lastHeading)*robotDeltaX-Math.sin(lastHeading)*robotDeltaY;
        deltaY = Math.sin(lastHeading)*robotDeltaX+Math.cos(lastHeading)*robotDeltaY;

        //returns the new global position by adding the changes in position
        poseEstimate = new Pose2d(
                poseEstimate.getX()+deltaX,
                poseEstimate.getY()+deltaY,
                currentHeading
        );

        //Sets the last positions and variables to the current for next loop iteration
        lastWheelPositions = currentWheelPositions;
        lastHeading = currentHeading;

    }

    /**
     * gets the wheel positions in inches
     * @return a list of wheel positions
     */
    public List<Double> getWheelPositions(){

        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null){ return Arrays.asList(0.0,0.0,0.0,0.0); }

        List<Double> positions = new ArrayList<>();
        positions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(leftFront)));
        positions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(leftRear)));
        positions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(rightRear)));
        positions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(rightFront)));

        return positions;
    }

    public double getHeading(){
        return (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + (2 * Math.PI) + headingOffset )%(2*Math.PI);
    }

    public void setPoseEstimate(Pose2d estimate){ poseEstimate = estimate; }

    /**
     * This is used to set the start pose
     * @param pose that is set as the start pose
     */
    public void setStartPose(Pose2d pose) {
        poseEstimate = pose;
        headingOffset = pose.getHeading();
        currentHeading = getHeading();
        lastHeading = getHeading();
    }

    public Pose2d getPoseEstimate(){ return poseEstimate; }
}