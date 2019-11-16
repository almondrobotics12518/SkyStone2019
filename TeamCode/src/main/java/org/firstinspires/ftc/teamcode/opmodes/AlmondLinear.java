package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import com.qualcomm.robotcore.util.ElapsedTime;
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
import org.firstinspires.ftc.teamcode.roadrunner.drive.tank.SampleMecanumDrive;
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

    public VuforiaTrackables targetsSkyStone;
    public SampleMecanumDrive drive;
    public Hook hook;
    public Claw claw;
    public long timeElapsed;

    public static final String VUFORIA_KEY =
            "AapPoTb/////AAABmcGNGhG7GUe/iZ1mnxUvFtiIlkU7ezYNDHjvlnApSPJtrWB9SWukzQuzeVOPBEgk1EIT1qr0HIXB7KdkXBiBakilo9wE4ya/P9MunTSV8dOe2wAEej6VZOeZF46YcDilT+LG3Fu1FJ2KmMJrgAjT/1P3k1KTSs4kuY0m+2nJK3foxjQNVGB+m7bRX9cQqhQeTJvE1Us4RyXekpmxBpbyEvj6gtVHq179S4PNyjs1r/a+jcX9amOfD8IkihmH3wYZR6VH8ryuDKAnFJ+RD/oqW4Aa8WwbAhnseXEG0OwKk1SX5G/yUrahz4S1dNjna5sj1yxfRepZVrKG4qOEmH+kfX+eTn3+ssPnKzodtbJr9ptm";
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public boolean targetVisible = false;
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

        this.targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        allTrackables.addAll(targetsSkyStone);
        targetsSkyStone.activate();

    }
    public void Nav() {



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

            if(isStopRequested()){
                this.targetsSkyStone.deactivate();
            }


    }

    public enum Positions {
        LEFT, MIDDLE, RIGHT, NONE
    }

    /**
     * this methos is used to turn to an absolute angle
     * @param degrees to turn in an absolute angle, Counter clockwise is positive and clockwise is negative
     */
    public void turn(double degrees){
        drive.turn(Angle.normDelta(Math.toRadians(degrees)-drive.getPoseEstimate().getHeading()));
        while(drive.isBusy()&&isStarted()&&!isStopRequested()){
            drive.update();
        }
    }

    public void intake(){
        ElapsedTime time = new ElapsedTime();

        driveSideways(0.5, 1000);
        turn(-90);

        hook.extend();
        time.reset();
        while (time.milliseconds() < 500 && !isStopRequested()) {
        }

        driveSideways(-0.5, 1000);
        turn(-90);
    }


    /**
     * This method is used to go forwards a speicied number of inches
     * @param inches to travel
     */
    public void forward(double inches){
        drive.followTrajectory(
                drive.trajectoryBuilder().
                        forward(inches).
                        build()
        );
        while(drive.isBusy()&&!isStopRequested()&&isStarted()){
            drive.update();
        }
    }


    /**
     * Moves a certain number of inches backwards
     * @param inches to go backwards
     */
    public void back(double inches){
        drive.followTrajectory(
                drive.trajectoryBuilder().
                        back(inches).
                        build()
        );
        while(drive.isBusy()&&!isStopRequested()&&isStarted()){
            drive.update();
        }
    }

    /**
     * Method to make the robot move sideways at a certain power for a time
     * @param power What power to set to the motors, left is positive, right is negative
     * @param millis How many milliseconds to set the power for
     */

    public void driveSideways(double power, int millis){
        ElapsedTime t = new ElapsedTime();
        t.reset();
        drive.setDrivePower(
                new Pose2d(0,power,0)
        );

        while(!isStopRequested() && isStarted() && t.milliseconds()<millis){
            drive.updatePoseEstimate();
        }

        drive.setDrivePower(new Pose2d());
    }
}

