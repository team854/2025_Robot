package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    // Spark MAX controllers for each joint
    private final SparkMax                  shoulderMotor;
    private final SparkMax                  wristMotor;

    // Closed-loop controllers and encoders for the shoulder joint
    private final SparkClosedLoopController shoulderClosedLoop;
    private final RelativeEncoder           shoulderEncoder;

    // Limit switch for wrist
    private final DigitalInput              wristLimitSwitch;

    // Setpoints (can be adjusted via ArmConstants or dynamically tuned)
    private double                          shoulderSetpoint;

    public ArmSubsystem() {
        // Initialize motors on the appropriate CAN IDs
        shoulderMotor      = new SparkMax(ArmConstants.SHOULDER_MOTOR_ID, MotorType.kBrushless);
        wristMotor         = new SparkMax(ArmConstants.WRIST_MOTOR_ID, MotorType.kBrushed);

        // Retrieve built-in closed-loop controllers and encoders for the shoulder
        shoulderClosedLoop = shoulderMotor.getClosedLoopController();
        shoulderEncoder    = shoulderMotor.getEncoder();

        // Initialize limit switch (assuming it's on a DIO port)
        wristLimitSwitch   = new DigitalInput(ArmConstants.WRIST_LIMIT_SWITCH_PORT);

        // Set up closed loop for shoulder
        SparkMaxConfig shoulderConfig = new SparkMaxConfig();

        // Set PID gains
        shoulderConfig.closedLoop
            .p(ArmConstants.kShoulderP)
            .i(ArmConstants.kShoulderI)
            .d(ArmConstants.kShoulderD)
            .velocityFF(1 / Constants.NEO_MOTOR_Kv_VALUE)
            .outputRange(ArmConstants.SHOULDER_MIN_OUTPUT, ArmConstants.SHOULDER_MAX_OUTPUT);

        // Set up REV MaxMotion config for the shoulder motor
        shoulderConfig.closedLoop.maxMotion
            .maxVelocity(ArmConstants.SHOULDER_MAX_VELOCITY)
            .maxAcceleration(ArmConstants.SHOULDER_MAX_ACCELERATION)
            .allowedClosedLoopError(ArmConstants.SHOULDER_ALLOWED_CLOSED_LOOP_ERROR);

        // Initialize setpoints to the current positions (or default positions as needed)
        shoulderSetpoint = shoulderEncoder.getPosition();
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
     * Moves the wrist to either 0 degrees or 90 degrees using open-loop control.
     * Assumes the hard stop or limit switch determines the position.
     *
     * @param toNinetyDegrees If true, move to 90 degrees; if false, move to 0 degrees.
     */
    public void moveWrist(boolean toNinetyDegrees) {
        if (toNinetyDegrees) {
            wristMotor.set(ArmConstants.WRIST_UP_SPEED);
        }
        else {
            wristMotor.set(ArmConstants.WRIST_DOWN_SPEED);
        }
    }

    /**
     * Stops the wrist motor.
     */
    public void stopWrist() {
        wristMotor.stopMotor();
    }

    /**
     * Stop all arm movement.
     */
    public void stop() {
        shoulderMotor.stopMotor();
        stopWrist();
    }

    @Override
    public void periodic() {
        // Publish current positions and setpoints to SmartDashboard for tuning and debugging
        SmartDashboard.putNumber("Arm/Shoulder Position", shoulderEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Shoulder Setpoint", shoulderSetpoint);
        SmartDashboard.putBoolean("Arm/Wrist Limit Switch", wristLimitSwitch.get());

        // Stop wrist if limit switch is triggered
        if (!wristLimitSwitch.get()) {
            stopWrist();
        }
    }
}
