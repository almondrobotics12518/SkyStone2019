package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class Detector {

    public CustomSkystoneDetector detector;
    public OpenCvCamera phoneCam;

    public Detector(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        detector = new CustomSkystoneDetector();
        detector.useDefaults();

        phoneCam.openCameraDevice();
        phoneCam.setPipeline(detector);
    }

    public void startStreaming(){
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

}
