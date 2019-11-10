package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleTankDriveBase;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Hook;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public abstract class AlmondLinear extends LinearOpMode {

    public SampleTankDriveBase drive;
    public Hook hook;
    public Claw claw;
    public static final String VUFORIA_KEY =
            "AapPoTb/////AAABmcGNGhG7GUe/iZ1mnxUvFtiIlkU7ezYNDHjvlnApSPJtrWB9SWukzQuzeVOPBEgk1EIT1qr0HIXB7KdkXBiBakilo9wE4ya/P9MunTSV8dOe2wAEej6VZOeZF46YcDilT+LG3Fu1FJ2KmMJrgAjT/1P3k1KTSs4kuY0m+2nJK3foxjQNVGB+m7bRX9cQqhQeTJvE1Us4RyXekpmxBpbyEvj6gtVHq179S4PNyjs1r/a+jcX9amOfD8IkihmH3wYZR6VH8ryuDKAnFJ+RD/oqW4Aa8WwbAhnseXEG0OwKk1SX5G/yUrahz4S1dNjna5sj1yxfRepZVrKG4qOEmH+kfX+eTn3+ssPnKzodtbJr9ptm";
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private boolean targetVisible = false;
    private OpenGLMatrix lastLocation = null;
    private static final float mmPerInch = 25.4f;
    public VuforiaLocalizer vuforia;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    //private TFObjectDetector tfod;


    /*public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();


        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.fillCameraMonitorViewParent = true;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void initTF() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        if (tfod != null) {
            tfod.activate();
        }
    }
*/
    public void initNav() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        allTrackables.addAll(targetsSkyStone);
        targetsSkyStone.activate();
        while (!isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getVuforiaCameraFromTarget();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();


    }

    public Positions autoPath() {

        // breakpoints for deciding paths
        VectorF translation = lastLocation.getTranslation();
        int rightRecognition = 600;
        int middleRecognition = 400;

        // inital path none
        Positions path = Positions.NONE;

        // if skystone is visible, decide path based on location of skystone
        if(targetVisible){
            if(translation.get(0)/mmPerInch > rightRecognition) {
                path = Positions.RIGHT;
                telemetry.addData("Going Right","none");
            } else if(translation.get(1)/mmPerInch > middleRecognition && translation.get(1) < rightRecognition) {
                path = Positions.MIDDLE;
                telemetry.addData("Going Middle","none");
            } else{
                path = Positions.LEFT;
                telemetry.addData("Going Left","none");
            }
        }
        telemetry.update();
        return path;
    }

    public boolean isDetected() {
        return targetVisible;
    }


    public enum Positions {
        LEFT, MIDDLE, RIGHT, NONE
    }


}

