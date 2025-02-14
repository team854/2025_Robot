package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    // Spark MAX controllers for each joint
    private final SparkMax                  shoulderMotor;
    private final SparkMax                  wristMotor;

    // Closed-loop controllers and encoders for each joint
    private final SparkClosedLoopController shoulderClosedLoop;
    private final SparkClosedLoopController wristClosedLoop;
    private final RelativeEncoder           shoulderEncoder;
    private final RelativeEncoder           wristEncoder;

    // Setpoints (can be adjusted via ArmConstants or dynamically tuned)
    private double                          shoulderSetpoint;
    private double                          wristSetpoint;

    public ArmSubsystem() {
        // Initialize motors on the appropriate CAN IDs
        shoulderMotor      = new SparkMax(ArmConstants.SHOULDER_MOTOR_ID, MotorType.kBrushless);
        wristMotor         = new SparkMax(ArmConstants.WRIST_MOTOR_ID, MotorType.kBrushless);

        // Retrieve built-in closed-loop controllers and encoders
        shoulderClosedLoop = shoulderMotor.getClosedLoopController();
        wristClosedLoop    = wristMotor.getClosedLoopController();

        shoulderEncoder    = shoulderMotor.getEncoder();
        wristEncoder       = wristMotor.getEncoder();

        // Set up closed loop for shoulder
        SparkMaxConfig shoulderConfig = new SparkMaxConfig();

        // Set PID gains
        shoulderConfig.closedLoop
            .p(ArmConstants.kShoulderP)
            .i(ArmConstants.kShoulderI)
            .d(ArmConstants.kShoulderD)
            .velocityFF(1 / Constants.NEO_MOTOR_Kv_VALUE) // Feed Forward value (reciprocal of the kV value for NEO V1.1 motor)
            .outputRange(ArmConstants.SHOULDER_MIN_OUTPUT, ArmConstants.SHOULDER_MAX_OUTPUT);


        // Set up REV MaxMotion config for the shoulder motor
        shoulderConfig.closedLoop.maxMotion
            .maxVelocity(ArmConstants.SHOULDER_MAX_VELOCITY)
            .maxAcceleration(ArmConstants.SHOULDER_MAX_ACCELERATION)
            .allowedClosedLoopError(ArmConstants.SHOULDER_ALLOWED_CLOSED_LOOP_ERROR);

        // Set up closed loop for wrist
        SparkMaxConfig wristConfig = new SparkMaxConfig();

        // Set PID gains
        wristConfig.closedLoop
            .p(ArmConstants.kWristP)
            .i(ArmConstants.kWristI)
            .d(ArmConstants.kWristD)
            .velocityFF(1 / Constants.NEO_MOTOR_Kv_VALUE) // Feed Forward value (reciprocal of the kV value for NEO V1.1 motor)
            .outputRange(ArmConstants.WRIST_MIN_OUTPUT, ArmConstants.WRIST_MAX_OUTPUT);


        // Set up REV MaxMotion config for the wrist motor
        wristConfig.closedLoop.maxMotion
            .maxVelocity(ArmConstants.WRIST_MAX_VELOCITY)
            .maxAcceleration(ArmConstants.WRIST_MAX_ACCELERATION)
            .allowedClosedLoopError(ArmConstants.WRIST_ALLOWED_CLOSED_LOOP_ERROR);

        // Initialize setpoints to the current positions (or default positions as needed)
        shoulderSetpoint = shoulderEncoder.getPosition();
        wristSetpoint    = wristEncoder.getPosition();
    }

    /**
     * Command the shoulder joint to move to the specified setpoint.
     *
     * @param setpoint The target position (encoder units or degrees, per your configuration)
     */
    public void moveShoulderToSetpoint(double setpoint) {
        shoulderSetpoint = setpoint;
        shoulderClosedLoop.setReference(shoulderSetpoint, ControlType.kMAXMotionPositionControl);
    }

    /**
     * Command the wrist joint to move to the specified setpoint.
     *
     * @param setpoint The target position (encoder units or degrees)
     */
    public void moveWristToSetpoint(double setpoint) {
        wristSetpoint = setpoint;
        wristClosedLoop.setReference(wristSetpoint, ControlType.kMAXMotionPositionControl);
    }

    /**
     * Stop all arm movement.
     */
    public void stop() {
        shoulderMotor.stopMotor();
        wristMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // Publish current positions and setpoints to SmartDashboard for tuning and debugging
        SmartDashboard.putNumber("Arm/Shoulder Position", shoulderEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Shoulder Setpoint", shoulderSetpoint);
        SmartDashboard.putNumber("Arm/Wrist Position", wristEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Wrist Setpoint", wristSetpoint);
    }
}
