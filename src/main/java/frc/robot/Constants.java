// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        public static final double DEADBAND                 = 0.04;
        public static final double SWERVE_TRANSLATION_SCALE = 0.75;
        public static final double SWERVE_ROTATION_SCALE    = 0.5;                     // Negative values invert right stick
        public static final double MAX_SPEED                = Units.feetToMeters(14.5);
        public static final double FINE_SPEED_REDUCTION     = Units.feetToMeters(2);
        public static final double FINE_ROTATION_REDUCTION  = 0.8;
        public static final double GYRO_OFFSET              = 180;

    }

    public static class AutoAlignConstants {

        public static final double ROT_SETPOINT_REEF_ALIGNMENT  = 0;
        public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0;
        public static final double X_SETPOINT_REEF_ALIGNMENT    = 0;
        public static final double X_TOLERANCE_REEF_ALIGNMENT   = 0;
        public static final double Y_SETPOINT_REEF_ALIGNMENT    = 0;
        public static final double Y_TOLERANCE_REEF_ALIGNMENT   = 0;
        public static final double DONT_SEE_TAG_WAIT_TIME       = 0;
        public static final double POSE_VALIDATION_TIME         = 0;
        public static final double X_REEF_ALIGNMENT_P           = 0;
        public static final double Y_REEF_ALIGNMENT_P           = 0;
        public static final double ROT_REEF_ALIGNMENT_P         = 0;

    }

    public class Tolerances {
        /*
         * Shoulder
         */
        public static final double SHOULDER_LOWER_TOLERANCE = 5;
        public static final double SHOULDER_UPPER_TOLERANCE = 5;
        /*
         * Elevator
         */
        public static final double ELEVATOR_LOWER_TOLERANCE = 7.0;
        public static final double ELEVATOR_UPPER_TOLERANCE = 7.0;
    }

    public static class ElevatorConstants {

        /*
         * Elevator CAN IDs
         */
        public static final int    LOWER_STAGE_MOTOR_CANID                = 3;
        public static final int    UPPER_STAGE_MOTOR_CANID                = 2;

        /*
         * Elevator encoder offsets
         */
        public static final double ELEVATOR_TOP_STAGE_ENCODER_ZERO_OFFSET = 0.0;  // Fix this


        /*
         * Elevator top stage PID
         */
        public static final double kUpperStageP                           = 0.5;
        public static final double kUpperStageI                           = 0.0;
        public static final double kUpperStageD                           = 0.1;
        public static final double UPPER_STAGE_MAX_VELOCITY               = 120.0;
        public static final double UPPER_STAGE_MAX_ACCELERATION           = 120.0;
        public static final double UPPER_STAGE_ALLOWED_CLOSED_LOOP_ERROR  = 0.1;

        /*
         * For manual control of top stage
         */
        public static final double ELEVATOR_TOP_STAGE_UP_SPEED            = 1.0;
        public static final double ELEVATOR_TOP_STAGE_DOWN_SPEED          = -1.0;

        /*
         * Elevator bottom stage PID
         */
        public static final double kLowerStageP                           = 0.5;
        public static final double kLowerStageI                           = 0.0;
        public static final double kLowerStageD                           = 0.2;
        public static final double LOWER_STAGE_MAX_VELOCITY               = 120;
        public static final double LOWER_STAGE_MAX_ACCELERATION           = 120;

        /*
         * For manual control of bottom stage
         */
        public static final double ELEVATOR_BOTTOM_STAGE_UP_SPEED         = 1.0;
        public static final double ELEVATOR_BOTTOM_STAGE_DOWN_SPEED       = -1.0;

        /*
         * Elevator Limits
         */
        public static final double ELEVATOR_UPPER_STAGE_UPPER_LIMIT       = 56;
        public static final double ELEVATOR_UPPER_STAGE_LOWER_LIMIT       = -53;
        public static final double ELEVATOR_LOWER_STAGE_UPPER_LIMIT       = 200;
        public static final double ELEVATOR_LOWER_STAGE_LOWER_LIMIT       = 3.5;


        /*
         * Max and Min Height (feet) for lower stage
         */
        public static final double LOWER_STAGE_MIN_HEIGHT                 = 0.0;
        public static final double LOWER_STAGE_MAX_HEIGHT                 = 3.0;

        /*
         * Elevator Ratios
         */

        // Gear ratios
        public static final double ELEVATOR_LOWER_GEAR_RATIO           = 20.0;
        public static final double ELEVATOR_UPPER_GEAR_RATIO           = 10.0;

        // Rotations to feet for the lower stage
        public static final double ELEVATOR_LOWER_DRUM_DIAMETER_INCHES = 2.25;
        public static final double ROTATIONS_TO_FEET_LOWER             = (Math.PI * ELEVATOR_LOWER_DRUM_DIAMETER_INCHES
            / 12.0)
            / ELEVATOR_LOWER_GEAR_RATIO;

        // Rotations to feet for the upper stage
        public static final double ELEVATOR_UPPER_DRUM_DIAMETER_INCHES = 2.25;
        public static final double ROTATIONS_TO_FEET_UPPER             = (Math.PI * ELEVATOR_UPPER_DRUM_DIAMETER_INCHES / 12
            / ELEVATOR_UPPER_GEAR_RATIO);

        /*
         * Elevator setpoints
         */
        public static final double UPPER_ELEVATOR_DEFAULT_SETPOINT     = 4.66;
        public static final double UPPER_ELEVATOR_GROUND_SETPOINT      = 4.66;
        public static final double UPPER_ELEVATOR_L1_SETPOINT          = 4.66;
        public static final double UPPER_ELEVATOR_L2_SETPOINT          = 55.8;
        public static final double UPPER_ELEVATOR_L3_SETPOINT          = -31.9;
        public static final double UPPER_ELEVATOR_L4_SETPOINT          = 200;
        public static final double UPPER_ELEVATOR_NET_SETPOINT         = 5.0;
        public static final double UPPER_ELEVATOR_PROCESSOR_SETPOINT   = 1.5;
        public static final double UPPER_ELEVATOR_SOURCE_SETPOINT      = 0;

        public static final double LOWER_ELEVATOR_DEFAULT_SETPOINT     = -1.85;
        public static final double LOWER_ELEVATOR_GROUND_SETPOINT      = -1.85;
        public static final double LOWER_ELEVATOR_L1_SETPOINT          = 30;
        public static final double LOWER_ELEVATOR_L2_SETPOINT          = 0;
        public static final double LOWER_ELEVATOR_L3_SETPOINT          = 0;
        public static final double LOWER_ELEVATOR_L4_SETPOINT          = 56;
        public static final double LOWER_ELEVATOR_NET_SETPOINT         = 5.0;
        public static final double LOWER_ELEVATOR_PROCESSOR_SETPOINT   = 1.5;
        public static final double LOWER_ELEVATOR_SOURCE_SETPOINT      = 0;


    }

    public static class ClimbConstants {

        /*
         * VictorSPX CAN ID
         */
        public static final int    CLIMB_MOTOR_ID   = 7;

        /*
         * Climb speeds (should be full speed)
         */
        public static final double CLIMB_UP_SPEED   = -1;
        public static final double CLIMB_DOWN_SPEED = 1;
    }

    public static class ArmConstants {

        /*
         * IDs and ports
         */
        public static final int    SHOULDER_MOTOR_ID                     = 4;
        public static final int    SHOULDER_FOLLOWER_ID                  = 8;
        public static final int    WRIST_MOTOR_ID                        = 5;
        public static final int    INTAKE_MOTOR_ID                       = 6;
        public static final int    INTAKE_SENSOR_PORT                    = 0;
        public static final double SHOULDER_GEAR_RATIO                   = 1;


        /*
         * Speed Limits
         */
        public static final double MAX_SHOULDER_UP_SPEED                 = 1.0;
        public static final double MAX_SHOULDER_DOWN_SPEED               = -0.15;
        public static final double MAX_DEGREES_PER_LOOP                  = 2.0;

        public static final double MAX_WRIST_SPEED                       = 1.0;

        /*
         * Wrist rotation angles
         */
        public static final double WRIST_VERTICAL_ANGLE                  = 25;
        public static final double WRIST_HORIZONTAL_ANGLE                = 0;


        public static final double SHOULDER_ABSOLUTE_ENCODER_ZERO_OFFSET = 91.9 / 360;


        /*
         * Arm/shoulder scoring angles
         */
        public static final double ARM_L1_ANGLE                          = 41;
        public static final double ARM_L2_ANGLE                          = 122;
        public static final double ARM_L3_ANGLE                          = 110;
        public static final double ARM_L4_ANGLE                          = 122;

        /*
         * Arm intake angles
         */
        public static final double ARM_SOURCE_ANGLE                      = 110;
        public static final double ARM_GROUND_ANGLE                      = 44;
        public static final double ARM_DEFAULT_ANGLE                     = 0;

        /*
         * Other arm angles
         */
        public static final double ARM_HORIZONTAL_ANGLE                  = 100;
        public static final double ARM_TOP_ANGLE                         = 175.0;

        /*
         * End Effector
         */
        public static final double INTAKE_GROUND_SPEED                   = 0.8;
        public static final double INTAKE_SOURCE_SPEED                   = 1.0;
        public static final double BRANCH_SCORE_SPEED                    = 0.4;
        public static final double TROUGH_SCORE_SPEED                    = 0.3;
        public static final double PROCESSOR_SCORE_SPEED                 = 1.0;

        /*
         * Values for shoulder
         */
        public static final double kShoulderP                            = 0.005;
        public static final double kShoulderI                            = 0.0;
        public static final double kShoulderD                            = 1;
        public static final double SHOULDER_OFFSET                       = 25;
        public static final double SHOULDER_MAX_VELOCITY                 = 120;
        public static final double SHOULDER_MAX_ACCELERATION             = 120;
        public static final double SHOULDER_ALLOWED_CLOSED_LOOP_ERROR    = 1.0;

        /*
         * Values for wrist
         */
        public static final double kWristP                               = 0.04;
        public static final double kWristI                               = 0.0;
        public static final double kWristD                               = 0.005;
        public static final double WRIST_GEAR_RATIO                      = 100.0;
        public static final double WRIST_MAX_VELOCITY                    = 50;
        public static final double WRIST_MAX_ACCELERATION                = 50;

    }

    public static class VisionConstants {

        /*
         * Different limelight names
         */
        public static final String LIMELIGHT_NAME1          = "limelight";

        /*
         * Limelight distance offsets (meters)
         * Based on robot centre point
         */
        public static final double LIMELIGHT_OFFSET_FORWARD = 0.0;
        public static final double LIMELIGHT_OFFSET_LEFT    = -0.4;
        public static final double LIMELIGHT_OFFSET_HEIGHT  = 0.9;

        /*
         * Limelight rotation offsets (degrees)
         * Based on robot forward orientation
         */
        public static final double LIMELIGHT_ROLL           = 0.0;
        public static final double LIMELIGHT_PITCH          = 0.0;
        public static final double LIMELIGHT_YAW            = -20;


        /* Conditions to check if vision measurement should be added */

        // Average distance of visible tags > this value (meters)
        public static final double  AVG_TAG_DIST_FILTER     = 5.0;

        // If there are more than this many tags visible
        public static final Integer MIN_TAGS_VISIBLE_FILTER = 1;

        // If average ambiguity between tags is less than this value
        public static final double  TAG_AMBIGUITY_FILTER    = 0.5;
    }

    /*
     * Absolute constant values (will not need to be changed)
     */
    public static final double NEO_MOTOR_Kv_VALUE = 473.0;
    public static final double NEO_550_Kv_VALUE   = 917.0;
    public static final double INCHES_TO_FEET     = 1.0 / 12.0;

}