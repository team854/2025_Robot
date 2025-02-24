// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int    kDriverControllerPort    = 0;
        public static final int    kOperatorControllerPort  = 1;
        public static final double DEADBAND                 = 0.05;
        public static final double OPERATOR_DEADBAND        = 0.05;
        public static final double SWERVE_TRANSLATION_SCALE = 1;
        public static final double SWERVE_ROTATION_SCALE    = 1.2;                  // Negative values invert right stick
        public static final double MAX_SPEED                = Units.feetToMeters(6);
    }

    public static class ElevatorConstants {

        public static final int      LOWER_STAGE_MOTOR_CANID          = 2;
        public static final int      UPPER_STAGE_MOTOR_CANID          = 3;

        // Bottom stage speeds set without a PID Controller
        public static final double   ELEVATOR_BOTTOM_STAGE_RISE_SPEED = 0.5;
        public static final double   ELEVATOR_BOTTOM_STAGE_FALL_SPEED = 0.5;

        // Elevator top stage PID
        public static final double   kElevatrorP                      = 0.0;
        public static final double   kElevatrorI                      = 0.0;
        public static final double   kElevatrorD                      = 0.0;

        // Elevator ratios
        public static final double   ELEVATOR_GEAR_RATIO              = 5.0;
        public static final double   ELEVATOR_DRUM_DIAMETER_INCHES    = 0.5;

        // Elevator setpoints
        public static final double   ELEVATOR_INTAKE_SETPOINT         = 0.0;
        public static final double   ELEVATOR_L1_SETPOINT             = 0.0;
        public static final double   ELEVATOR_L2_SETPOINT             = 0.0;
        public static final double   ELEVATOR_L3_SETPOINT             = 0.0;
        public static final double   ELEVATOR_L4_SETPOINT             = 0.0;
        public static final double   ELEVATOR_NET_SETPOINT            = 0.0;
        public static final double   ELEVATOR_PROCESSOR_SETPOINT      = 0.0;

        // All setpoints for elevator
        public static final double[] ELEVATOR_UPPER_STAGE_SETPOINTS   = {
                ELEVATOR_INTAKE_SETPOINT,
                ELEVATOR_L1_SETPOINT,
                ELEVATOR_L2_SETPOINT,
                ELEVATOR_L3_SETPOINT,
                ELEVATOR_L4_SETPOINT,
                ELEVATOR_NET_SETPOINT,
                ELEVATOR_PROCESSOR_SETPOINT };
    }

    public static class ArmConstants {
        public static final int    SHOULDER_MOTOR_ID                  = 0;
        public static final int    WRIST_MOTOR_ID                     = 1;
        public static final int    WRIST_LIMIT_SWITCH_PORT            = 0;

        // Shoulder PID Values
        public static final double kShoulderP                         = 0.0;
        public static final double kShoulderI                         = 0.0;
        public static final double kShoulderD                         = 0.0;

        // Shoulder Closed Loop Config
        public static final double SHOULDER_MAX_OUTPUT                = 1.0;
        public static final double SHOULDER_MIN_OUTPUT                = -1.0;

        // Shoulder MAXMotion Config
        public static final double SHOULDER_MAX_VELOCITY              = 1.0;
        public static final double SHOULDER_MAX_ACCELERATION          = 1.0;
        public static final double SHOULDER_ALLOWED_CLOSED_LOOP_ERROR = 1.0;

        // Wrist speeds
        public static final double WRIST_UP_SPEED                     = 0.1;
        public static final double WRIST_DOWN_SPEED                   = 0.1;

    }

    public static class VisionConstants {

        public static final String     LIMELIGHT_NAME1           = "Limelight 4";
        public static final String     LIMELIGHT_NAME2           = "Limelight 3G";
        public static final String     LIMELIGHT_NAME3           = "Limelight 3";

        // In Meters
        public static final double     LIMELIGHT_OFFSET_X        = 0.0;
        public static final double     LIMELIGHT_OFFSET_Y        = 0.0;
        public static final double     LIMELIGHT_OFFSET_Z        = 0.0;

        // Degrees
        public static final double     LIMELIGHT_ROTATION_X      = 0.0;
        public static final double     LIMELIGHT_ROTATION_Y      = 0.0;
        public static final double     LIMELIGHT_ROTATION_Z      = 0.0;
        public static final Rotation3d LIMELIGHT_OFFSET_ROTATION = new Rotation3d(LIMELIGHT_ROTATION_X, LIMELIGHT_OFFSET_Y,
            LIMELIGHT_ROTATION_Z);

        /* Conditions to check if vision measurement should be added */

        // Average distance of visible tags > this value (meters)
        public static final double  AVG_TAG_DIST_FILTER     = 5.0;

        // If there are more than this many tags visible
        public static final Integer MIN_TAGS_VISIBLE_FILTER = 0;

        // If average ambiguity between tags is less than this value
        public static final double  TAG_AMBIGUITY_FILTER    = 0.3;
    }

    public static final double NEO_MOTOR_Kv_VALUE = 473.0;
    public static final double INCHES_TO_FEET     = 1.0 / 12.0;
}