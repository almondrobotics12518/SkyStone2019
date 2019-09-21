package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    DcMotor intakeLeft;
    DcMotor intakeRight;

    public Intake(HardwareMap  hardwareMap) {
            intakeLeft = hardwareMap.dcMotor.get("IntakeLeft");
            intakeRight = hardwareMap.dcMotor.get("IntakeRight");

            intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }


}
