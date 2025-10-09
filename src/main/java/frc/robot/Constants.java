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
        public static final int    kDriverControllerPort               = 0;
        public static final int    kOperatorControllerPort             = 1;
        public static final double DEADBAND                            = 0.04;
        public static final double SWERVE_TRANSLATION_SCALE    = 1.0;//1.0;
        public static final double SWERVE_ROTATION_SCALE               = 0.6;                     // Negative values invert right
                                                                                                  // stick
        public static final double MAX_SPEED                = Units.feetToMeters(14.5); //0.05;
        public static final double GYRO_OFFSET                         = 180;
    }

    public class Tolerances {

        public static final double SHOULDER_LOWER_TOLERANCE = 5;
        public static final double SHOULDER_UPPER_TOLERANCE = 5;

        public static final double ELEVATOR_LOWER_TOLERANCE = 7.0;
        public static final double ELEVATOR_UPPER_TOLERANCE = 7.0;
    }

    public static class ElevatorConstants {

        public static final int    LOWER_STAGE_MOTOR_CANID                = 3;
        public static final int    UPPER_STAGE_MOTOR_CANID                = 2;

        public static final double ELEVATOR_TOP_STAGE_ENCODER_ZERO_OFFSET = 0.0;

        // Manual control
        public static final double ELEVATOR_TOP_STAGE_UP_SPEED            = 1.0;
        public static final double ELEVATOR_TOP_STAGE_DOWN_SPEED          = -1.0;
        public static final double ELEVATOR_BOTTOM_STAGE_UP_SPEED         = 1.0;
        public static final double ELEVATOR_BOTTOM_STAGE_DOWN_SPEED       = -1.0;

        // Elevator limits
        public static final double ELEVATOR_UPPER_STAGE_UPPER_LIMIT       = 56;
        public static final double ELEVATOR_UPPER_STAGE_LOWER_LIMIT       = -53;
        public static final double ELEVATOR_LOWER_STAGE_UPPER_LIMIT       = 200;
        public static final double ELEVATOR_LOWER_STAGE_LOWER_LIMIT       = 3.5;
    }

    public static class ClimbConstants {

        public static final int    CLIMB_MOTOR_ID = 7;
        public static final double CLIMB_UP_SPEED = -1;
    }

    public static class ArmConstants {

        // CAN IDs and ports
        public static final int    SHOULDER_MOTOR_ID                     = 4;
        public static final int    WRIST_MOTOR_ID                        = 5;
        public static final int    INTAKE_MOTOR_ID                       = 6;
        public static final int    INTAKE_SENSOR_PORT                    = 0;
        public static final double SHOULDER_GEAR_RATIO                   = 1;


        // Speeds for arm controller
        public static final double MAX_SHOULDER_UP_SPEED                 = 1.0;
        public static final double MAX_SHOULDER_DOWN_SPEED               = -0.15;
        public static final double MAX_DEGREES_PER_LOOP                  = 2.0;
        public static final double MAX_WRIST_SPEED                       = 1.0;

        // Wrist encoder positions
        public static final double WRIST_VERTICAL_ANGLE                  = 25;
        public static final double WRIST_HORIZONTAL_ANGLE                = 0;

        // Main shoulder encoder offset
        public static final double SHOULDER_ABSOLUTE_ENCODER_ZERO_OFFSET = 91.9 / 360;


        /*
         * -------------------TUNE AT FIELD CALIBRATION-------------------
         * Bring robot to the reef on the field and line the arm up a few eaches above each branch of the reef
         * Enter the smart dashboard values for the arm angle into these constants
         * Notes: L2-L4 should all be relatively the same, when measuring put the elevator in the position where the arm gets the
         * best angle of attack on the reef branch, always better to be a few inches higher than needed since being too low makes
         * it incredibly hard to score whereas too high is a very easy adjustment for the driver
         */
        public static final double ARM_L1_ANGLE                          = 41;        // 41
        public static final double ARM_L2_ANGLE                          = 122;       // 122
        public static final double ARM_L3_ANGLE                          = 110;       // 110
        public static final double ARM_L4_ANGLE                          = 122;       // 122

        public static final double ARM_SOURCE_ANGLE                      = 110;       // 110
        public static final double ARM_GROUND_ANGLE                      = 44;        // 44
        public static final double ARM_DEFAULT_ANGLE                     = 0;         // 0

        /*
         * Other arm angles can be tuned if needed
         */
        public static final double ARM_HORIZONTAL_ANGLE                  = 100;
        public static final double ARM_TOP_ANGLE                         = 175.0;

        // End effector speeds
        public static final double INTAKE_GROUND_SPEED                   = 0.6;
        public static final double INTAKE_SOURCE_SPEED                   = 1.0;
        public static final double BRANCH_SCORE_SPEED                    = 0.4;
        public static final double TROUGH_SCORE_SPEED                    = 0.3;

        // Shoulder proportional controller
        public static final double kShoulderP                            = 0.005;
        public static final double SHOULDER_OFFSET                       = 25;

        // Wrist proportional controller
        public static final double kWristP                               = 0.04;
        public static final double WRIST_GEAR_RATIO                      = 100.0;
    }

    public static class VisionConstants {

        // Limelight name
        public static final String LIMELIGHT_NAME1          = "limelight";

        // Limelight position (OUT OF DATE)
        public static final double LIMELIGHT_OFFSET_FORWARD = 0.0;
        public static final double LIMELIGHT_OFFSET_LEFT    = -0.4;
        public static final double LIMELIGHT_OFFSET_HEIGHT  = 0.9;
        public static final double LIMELIGHT_ROLL           = 0.0;
        public static final double LIMELIGHT_PITCH          = 0.0;
        public static final double LIMELIGHT_YAW            = -20;
    }

    // Global constants if needed
    public static final double NEO_MOTOR_Kv_VALUE = 473.0;
    public static final double NEO_550_Kv_VALUE   = 917.0;
    public static final double INCHES_TO_FEET     = 1.0 / 12.0;

}