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
        public static final double DEADBAND                 = 0.05;
        public static final double SWERVE_TRANSLATION_SCALE = 1;
        public static final double SWERVE_ROTATION_SCALE    = 1.2;                  // Negative values invert right stick
        public static final double MAX_SPEED                = Units.feetToMeters(6);
    }

    public static class ElevatorConstants {
        public static final double ELEVATOR_BOTTOM_STAGE_RISE_SPEED = 0.5;
        public static final double ELEVATOR_BOTTOM_STAGE_FALL_SPEED = 0.5;
    }

    public static class ArmConstants {
        public static final int    SHOULDER_MOTOR_ID                  = 0;
        public static final int    WRIST_MOTOR_ID                     = 1;

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

        // Wrist PID Values
        public static final double kWristP                            = 0.0;
        public static final double kWristI                            = 0.0;
        public static final double kWristD                            = 0.0;

        // Wrist Closed Loop Config
        public static final double WRIST_MAX_OUTPUT                   = 1.0;
        public static final double WRIST_MIN_OUTPUT                   = -1.0;

        // Wrist MAXMotion Config
        public static final double WRIST_MAX_VELOCITY                 = 1.0;
        public static final double WRIST_MAX_ACCELERATION             = 1.0;
        public static final double WRIST_ALLOWED_CLOSED_LOOP_ERROR    = 1.0;
    }

    public static final double NEO_MOTOR_Kv_VALUE = 473.0; // This value will not change
}