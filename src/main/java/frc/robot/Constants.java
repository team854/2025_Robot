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

    public class Tolerances {
        // Commands
        public static final double SHOULDER_LOWER_TOLERANCE = 1.0;
        public static final double SHOULDER_UPPER_TOLERANCE = 1.0;
        public static final double ELEVATOR_LOWER_TOLERANCE = 1.0;
        public static final double ELEVATOR_UPPER_TOLERANCE = 1.0;

        // Subsystems
        public static final double ELEVATOR_PID_TOLERANCE   = 0.05;
    }

    public static class ElevatorConstants {

        public static final int      LOWER_STAGE_MOTOR_CANID               = 2;
        public static final int      UPPER_STAGE_MOTOR_CANID               = 3;

        // Bottom stage speeds set without a PID Controller
        public static final double   ELEVATOR_BOTTOM_STAGE_RISE_SPEED      = 0.8;
        public static final double   ELEVATOR_BOTTOM_STAGE_FALL_SPEED      = -0.4;

        // Elevator top stage PID
        public static final double   kUpperStageP                          = 0.1;
        public static final double   kUpperStageI                          = 0.0;
        public static final double   kUpperStageD                          = 0.01;
        public static final double   UPPER_STAGE_MAX_OUTPUT                = 1.0;
        public static final double   UPPER_STAGE_MIN_OUTPUT                = -1.0;
        public static final double   UPPER_STAGE_MAX_VELOCITY              = 60.0;
        public static final double   UPPER_STAGE_MAX_ACCELERATION          = 120.0;
        public static final double   UPPER_STAGE_ALLOWED_CLOSED_LOOP_ERROR = 0.1;

        // Elevator bottom stage PID
        public static final double   kLowerStageP                          = 0.1;
        public static final double   kLowerStageI                          = 0.0;
        public static final double   kLowerStageD                          = 0.01;
        public static final double   LOWER_STAGE_MIN_OUTPUT                = -1.0;
        public static final double   LOWER_STAGE_MAX_OUTPUT                = 1.0;

        // Max and Min Height (feet) for lower stage
        public static final double   LOWER_STAGE_MIN_HEIGHT                = 0.0;
        public static final double   LOWER_STAGE_MAX_HEIGHT                = 3.0;

        // Elevator ratios
        public static final double   ELEVATOR_GEAR_RATIO                   = 10.0;
        public static final double   ELEVATOR_DRUM_DIAMETER_INCHES         = 2.25;
        public static final double   ROTATIONS_TO_FEET                     = (Math.PI * ELEVATOR_DRUM_DIAMETER_INCHES / 12.0)
            / ELEVATOR_GEAR_RATIO;

        // Elevator setpoints
        public static final double   ELEVATOR_DEFAULT_SETPOINT             = 0.0;
        public static final double   ELEVATOR_GROUND_SETPOINT              = 0.0;
        public static final double   ELEVATOR_L1_SETPOINT                  = 1.0;
        public static final double   ELEVATOR_L2_SETPOINT                  = 2.0;
        public static final double   ELEVATOR_L3_SETPOINT                  = 3.0;
        public static final double   ELEVATOR_L4_SETPOINT                  = 4.0;
        public static final double   ELEVATOR_NET_SETPOINT                 = 5.0;
        public static final double   ELEVATOR_PROCESSOR_SETPOINT           = 1.5;

        // All setpoints for elevator
        public static final double[] ELEVATOR_UPPER_STAGE_SETPOINTS        = {
                ELEVATOR_L1_SETPOINT,
                ELEVATOR_L2_SETPOINT,
                ELEVATOR_L3_SETPOINT,
                ELEVATOR_L4_SETPOINT,
                ELEVATOR_NET_SETPOINT,
                ELEVATOR_PROCESSOR_SETPOINT };
    }

    public static class ClimbConstants {
        public static final int    CLIMB_MOTOR_ID = 0;
        public static final double CLIMB_SPEED    = 0.1;
    }

    public static class ArmConstants {

        // Arm Motor IDS
        public static final int    SHOULDER_MOTOR_ID                  = 0;
        public static final int    WRIST_MOTOR_ID                     = 1;
        public static final int    INTAKE_MOTOR_ID                    = 2;

        // Arm Scoring Angles
        public static final double ARM_L1_ANGLE                       = 20;
        public static final double ARM_L2_ANGLE                       = -10;
        public static final double ARM_L3_ANGLE                       = 20;
        public static final double ARM_L4_ANGLE                       = 45;

        // Wrist Rotation Angles
        public static final double WRIST_VERTICAL_DEGREES             = 90.0;
        public static final double WRIST_HORIZONTAL_DEGREES           = 0.0;

        // Arm Intake Angles
        public static final double ARM_SOURCE_ANGLE                   = 45;
        public static final double ARM_GROUND_ANGLE                   = -45;
        public static final double ARM_DEFAULT_ANGLE                  = 0;

        // End Effector
        public static final double INTAKE_GROUND_SPEED                = 1.0;
        public static final double INTAKE_SOURCE_SPEED                = 0.9;
        public static final double BRANCH_SCORE_SPEED                 = 0.5;
        public static final double TROUGH_SCORE_SPEED                 = 0.3;
        public static final double PROCESSOR_SCORE_SPEED              = 1.0;

        // Shoulder PID Values
        public static final double kShoulderP                         = 0.0;
        public static final double kShoulderI                         = 0.0;
        public static final double kShoulderD                         = 0.0;

        // Shoulder Closed Loop Config
        public static final double SHOULDER_MAX_OUTPUT                = 1.0;
        public static final double SHOULDER_MIN_OUTPUT                = -1.0;

        // Wrist PID Values
        public static final double kWristP                            = 0.0;
        public static final double kWristI                            = 0.0;
        public static final double kWristD                            = 0.0;

        // Wrist Closed Loop Config
        public static final double WRIST_MAX_OUTPUT                   = 1.0;
        public static final double WRIST_MIN_OUTPUT                   = -1.0;
        public static final double WRIST_GEAR_RATIO                   = 32.0;

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
    public static final double NEO_550_Kv_VALUE   = 917.0;
    public static final double INCHES_TO_FEET     = 1.0 / 12.0;
}