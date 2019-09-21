package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.List;

public abstract class AlmondLinear extends LinearOpMode {

    public Intake intake = new Intake(hardwareMap);

    public static final String VUFORIA_KEY =
            "AapPoTb/////AAABmcGNGhG7GUe/iZ1mnxUvFtiIlkU7ezYNDHjvlnApSPJtrWB9SWukzQuzeVOPBEgk1EIT1qr0HIXB7KdkXBiBakilo9wE4ya/P9MunTSV8dOe2wAEej6VZOeZF46YcDilT+LG3Fu1FJ2KmMJrgAjT/1P3k1KTSs4kuY0m+2nJK3foxjQNVGB+m7bRX9cQqhQeTJvE1Us4RyXekpmxBpbyEvj6gtVHq179S4PNyjs1r/a+jcX9amOfD8IkihmH3wYZR6VH8ryuDKAnFJ+RD/oqW4Aa8WwbAhnseXEG0OwKk1SX5G/yUrahz4S1dNjna5sj1yxfRepZVrKG4qOEmH+kfX+eTn3+ssPnKzodtbJr9ptm";
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    public VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    public void initVoforia() {
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
    }


    public enum Positions {
        LEFT, MIDDLE, RIGHT;
    }

    public Positions autoPath() {
        int leftRecognition = 69;
        int middleRecognition = 900;

        Positions path = Positions.MIDDLE;

        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel() == "Skystone") {
                    double middle = (recognition.getRight() + recognition.getLeft()) / 2;
                    if (middle < leftRecognition) {
                    } else if (middle > leftRecognition || middle < middleRecognition) {
                        path = Positions.MIDDLE;
                    }
                }
            }
        } else {
            path = Positions.RIGHT;
        }
        return path;
    }
}
