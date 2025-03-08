package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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

    // Controller for the intake wheels
    private final VictorSPX                 intakeMotor;

    // Intake Sensor
    private final DigitalInput              intakeSensor;

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

        intakeSensor       = new DigitalInput(ArmConstants.INTAKE_SENSOR_PORT);

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

        // Burn the config to the SparkMax
        shoulderMotor.configure(shoulderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure wrist motor
        SparkMaxConfig wristConfig = new SparkMaxConfig();
        wristConfig.closedLoop
            .p(ArmConstants.kWristP)
            .i(ArmConstants.kWristI)
            .d(ArmConstants.kWristD)
            .velocityFF(1 / Constants.NEO_550_Kv_VALUE)
            .outputRange(ArmConstants.WRIST_MIN_OUTPUT, ArmConstants.WRIST_MAX_OUTPUT);

        // Burn the config to the SparkMax
        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize setpoints
        shoulderSetpoint = shoulderEncoder.getPosition();
        wristSetpoint    = wristEncoder.getPosition();
    }

    public void moveShoulderToSetpoint(double setpoint) {
        shoulderSetpoint = setpoint / (360 * ArmConstants.SHOULDER_GEAR_RATIO);
        shoulderClosedLoop.setReference(shoulderSetpoint, ControlType.kMAXMotionPositionControl);
    }

    public void moveWristToSetpoint(double degrees) {
        wristSetpoint = degrees / (360 * ArmConstants.WRIST_GEAR_RATIO);
        wristClosedLoop.setReference(wristSetpoint, ControlType.kPosition);
    }

    public boolean hasGamePiece() {
        return !intakeSensor.get(); // Assuming active-low signal (true when object detected)
    }

    public void setIntakeSpeed(double intakeSpeed, boolean isReversed) {
        if (!hasGamePiece()) {
            intakeMotor.set(VictorSPXControlMode.PercentOutput, isReversed ? -intakeSpeed : intakeSpeed);
            System.out.println("Intaking subsystem");
        }
        else {
            intakeMotor.set(VictorSPXControlMode.PercentOutput, 0); // Stop intake if a piece is detected
        }
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
        SmartDashboard.putBoolean("Intake/Game Piece Detected", hasGamePiece());
        SmartDashboard.putNumber("Arm/Shoulder Position", shoulderEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Shoulder Setpoint", shoulderSetpoint);
        SmartDashboard.putNumber("Arm/Wrist Position", wristEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Wrist Setpoint", wristSetpoint);
    }
}
