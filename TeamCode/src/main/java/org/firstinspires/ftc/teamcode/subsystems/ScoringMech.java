package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ScoringMech {
    private Arm elbow;
    private Arm wrist;

    public ScoringMech(HardwareMap hardwareMap){
        elbow = new Arm(hardwareMap.get(DcMotor.class,"elbow"),0,0);
        wrist = new Arm(hardwareMap.get(DcMotor.class, "wrist"),0,0);

    }
}
