package org.firstinspires.ftc.teamcode.roadrunner.drive.tank;

import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.roadrunner.drive.localizer.NewLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.util.AxesSigns;
import org.firstinspires.ftc.teamcode.roadrunner.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxOptimizedI2cFactory;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.encoderTicksToInches;

/*
 * Optimized tank drive implementation for REV ExHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */
public class SampleTankDriveREVOptimized extends SampleTankDriveBase {
    private ExpansionHubEx hub;
    private ExpansionHubEx hub2;
    public List<ExpansionHubMotor> motors, leftMotors, rightMotors;
    private BNO055IMU imu;
    public ExpansionHubMotor leftRear, leftFront, rightRear, rightFront;

    public SampleTankDriveREVOptimized(HardwareMap hardwareMap) {
        super();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        // TODO: adjust the names of the following hardware devices to match your configuration
        // for simplicity, we assume that the desired IMU and drive motors are on the same hub
        // if your motors are split between hubs, **you will need to add another bulk read**
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");


        /*imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        while(!imu.isGyroCalibrated()){
        }*/

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);



        // add/remove motors depending on your robot (e.g., 6WD)
        leftFront = hardwareMap.get(ExpansionHubMotor.class, "leftFront");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "leftRear");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "rightRear");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        leftMotors = Arrays.asList(leftFront, leftRear);
        rightMotors = Arrays.asList(rightFront, rightRear);

        for (ExpansionHubMotor motor : motors) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, comment out the following line
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        // TODO: set the tuned coefficients from DriveVelocityPIDTuner if using RUN_USING_ENCODER
        setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(25,1,0));

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        setLocalizer(new NewLocalizer(hardwareMap));

    }



    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftMotors.get(0).getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }



    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        double leftSum = 0, rightSum = 0;
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0);
        }

        for (DcMotorEx leftMotor : leftMotors) {
            leftSum += encoderTicksToInches(bulkData.getMotorCurrentPosition(leftMotor));
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightSum += encoderTicksToInches(bulkData.getMotorCurrentPosition(rightMotor));
        }
        return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size());
    }


    public void setMotorPowers(double v, double v1) {
        for (ExpansionHubMotor leftMotor : leftMotors) {
            leftMotor.setPower(v);
        }
        for (ExpansionHubMotor rightMotor : rightMotors) {
            rightMotor.setPower(v1);
        }
    }

    @Override
    public double getRawExternalHeading() {
        return getLocalizer().getPoseEstimate().getHeading();
    }

    @Override
    public void setMotorPowers(double first, double second, double third, double fourth){
        List<Double> powers = Arrays.asList(first,second,third,fourth);
        for(int i = 0; i<4; i++){
            motors.get(i).setPower(powers.get(i));
        }
    }

}
