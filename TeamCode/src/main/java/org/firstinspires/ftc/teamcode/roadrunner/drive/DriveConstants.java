package org.firstinspires.ftc.teamcode.roadrunner.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/*
 * Constants used for feedforward control of the robot
 */
@Config
public class DriveConstants {

    //non-velo pid kV = 0.03 , kStatic = 0.12  , kA =  0.0006  .

    private static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    private static final double TICKS_PER_REV = 537.6;

    public static double WHEEL_RADIUS = 2;
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 14.3;

    /*
     * Tune these!!!!
     */

    public static double kV = 0.018;
    public static double kA = 0.0035;
    public static double kStatic = 0.002;


    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            45.0, 35.0, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );


    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        return 312;
    }
}
