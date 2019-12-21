package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;


@TeleOp
public class AlmondPipeline extends LinearOpMode {

    OpenCvCamera phoneCam;
    CustomSkystoneDetector detector;

    @Override
    public void runOpMode() {

        // define camera/detector
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        detector = new CustomSkystoneDetector();
        detector.useDefaults();

        phoneCam.openCameraDevice();
        phoneCam.setPipeline(detector);


        waitForStart();

        // turn on camera
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        while(opModeIsActive()){
           telemetry.addLine(String.valueOf(detector.foundRectangle().x+detector.foundRectangle().width/2));
           telemetry.update();
        }

        if(!opModeIsActive()){
            phoneCam.stopStreaming();
        }


    }


    class Pipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input){

            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            return input;

        }
    }




}


