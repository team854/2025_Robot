package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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

    // Controller for the intake wheels
    private final VictorSPX                 intakeMotor;

    // Closed-loop controllers and encoders
    private final SparkClosedLoopController shoulderClosedLoop;
    private final RelativeEncoder           shoulderEncoder;
    private final RelativeEncoder           wristEncoder;
    private final SparkClosedLoopController wristClosedLoop;

    // Setpoints
    private double                          shoulderSetpoint;
    private double                          wristSetpoint;

    public ArmSubsystem() {
        // Initialize motors
        shoulderMotor      = new SparkMax(ArmConstants.SHOULDER_MOTOR_ID, MotorType.kBrushless);
        wristMotor         = new SparkMax(ArmConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
        intakeMotor        = new VictorSPX(ArmConstants.INTAKE_MOTOR_ID);

        // Retrieve controllers and encoders
        shoulderClosedLoop = shoulderMotor.getClosedLoopController();
        shoulderEncoder    = shoulderMotor.getEncoder();
        wristClosedLoop    = wristMotor.getClosedLoopController();
        wristEncoder       = wristMotor.getEncoder();

        // Configure shoulder motor
        SparkMaxConfig shoulderConfig = new SparkMaxConfig();
        shoulderConfig.closedLoop
            .p(ArmConstants.kShoulderP)
            .i(ArmConstants.kShoulderI)
            .d(ArmConstants.kShoulderD)
            .velocityFF(1 / Constants.NEO_MOTOR_Kv_VALUE)
            .outputRange(ArmConstants.SHOULDER_MIN_OUTPUT, ArmConstants.SHOULDER_MAX_OUTPUT);

        shoulderConfig.closedLoop.maxMotion
            .maxVelocity(ArmConstants.SHOULDER_MAX_VELOCITY)
            .maxAcceleration(ArmConstants.SHOULDER_MAX_ACCELERATION)
            .allowedClosedLoopError(ArmConstants.SHOULDER_ALLOWED_CLOSED_LOOP_ERROR);

        // Configure wrist motor
        SparkMaxConfig wristConfig = new SparkMaxConfig();
        wristConfig.closedLoop
            .p(ArmConstants.kWristP)
            .i(ArmConstants.kWristI)
            .d(ArmConstants.kWristD)
            .velocityFF(1 / Constants.NEO_550_Kv_VALUE)
            .outputRange(ArmConstants.WRIST_MIN_OUTPUT, ArmConstants.WRIST_MAX_OUTPUT);

        // Initialize setpoints
        shoulderSetpoint = shoulderEncoder.getPosition();
        wristSetpoint    = wristEncoder.getPosition();
    }

    public void moveShoulderToSetpoint(double setpoint) {
        shoulderSetpoint = setpoint;
        shoulderClosedLoop.setReference(shoulderSetpoint, ControlType.kMAXMotionPositionControl);
    }

    public void moveWristToSetpoint(double degrees) {
        wristSetpoint = degrees / (360 * ArmConstants.WRIST_GEAR_RATIO);
        wristClosedLoop.setReference(wristSetpoint, ControlType.kPosition);
    }

    public void setIntakeSpeed(double intakeSpeed, boolean isReversed) {
        intakeMotor.set(VictorSPXControlMode.Velocity, isReversed ? -intakeSpeed : intakeSpeed);
    }

    public void stopWrist() {
        wristMotor.stopMotor();
    }

    public void stop() {
        shoulderMotor.stopMotor();
        stopWrist();
    }

    public double getShoulderEncoderPosition() {
        return shoulderEncoder.getPosition();
    }

    public double getWristEncoderPosition() {
        return wristEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm/Shoulder Position", shoulderEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Shoulder Setpoint", shoulderSetpoint);
        SmartDashboard.putNumber("Arm/Wrist Position", wristEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Wrist Setpoint", wristSetpoint);
    }
}
