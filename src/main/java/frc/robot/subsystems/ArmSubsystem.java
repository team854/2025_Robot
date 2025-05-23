package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    // Motor controllers for shoulder and wrist
    private final SparkMax              shoulderMotor;
    // private final SparkMax shoulderFollower;
    private final SparkMax              wristMotor;

    // Controller for the intake wheels
    private final VictorSPX             intakeMotor;

    // Intake Sensor
    private final DigitalInput          intakeSensor;

    // Encoders
    private final RelativeEncoder       shoulderEncoder;
    private final RelativeEncoder       wristEncoder;
    private final AbsoluteEncoder       shoulderAbsoluteEncoder;

    // WPILib Profiled PID controllers with trapezoidal constraints
    private final ProfiledPIDController shoulderController;
    private final ProfiledPIDController wristController;

    // Desired setpoints (in motor rotations)
    private double                      shoulderSetpoint;
    private double                      wristSetpoint;

    private boolean                     isGroundLock;

    public ArmSubsystem() {
        // Initialize motors
        shoulderMotor           = new SparkMax(ArmConstants.SHOULDER_MOTOR_ID, MotorType.kBrushless);
        // shoulderFollower = new SparkMax(ArmConstants.SHOULDER_FOLLOWER_ID, MotorType.kBrushless);
        wristMotor              = new SparkMax(ArmConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
        intakeMotor             = new VictorSPX(ArmConstants.INTAKE_MOTOR_ID);

        // Retrieve encoders
        shoulderEncoder         = shoulderMotor.getEncoder();
        shoulderAbsoluteEncoder = shoulderMotor.getAbsoluteEncoder();
        wristEncoder            = wristMotor.getEncoder();

        // Initialize intake sensor
        intakeSensor            = new DigitalInput(ArmConstants.INTAKE_SENSOR_PORT);

        /*
         * Shoulder Config
         */
        // Configure SparkMax controllers (basic configuration)
        SparkMaxConfig shoulderConfig = new SparkMaxConfig();

        // Optionally, set additional configuration options such as idle mode here.
        shoulderConfig.idleMode(IdleMode.kBrake);
        shoulderConfig.inverted(false);
        shoulderConfig.absoluteEncoder.zeroOffset(ArmConstants.SHOULDER_ABSOLUTE_ENCODER_ZERO_OFFSET);
        shoulderConfig.absoluteEncoder.inverted(false);

        shoulderMotor.configure(shoulderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // SparkMaxConfig followerConfig = new SparkMaxConfig();
        // followerConfig.idleMode(IdleMode.kBrake);
        // followerConfig.inverted(true);
        // shoulderFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        /*
         * Wrist Config
         */
        SparkMaxConfig wristConfig = new SparkMaxConfig();
        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Define trapezoidal motion profile constraints for shoulder and wrist.
        TrapezoidProfile.Constraints shoulderConstraints = new TrapezoidProfile.Constraints(
            ArmConstants.SHOULDER_MAX_VELOCITY,
            ArmConstants.SHOULDER_MAX_ACCELERATION);
        TrapezoidProfile.Constraints wristConstraints    = new TrapezoidProfile.Constraints(
            ArmConstants.WRIST_MAX_VELOCITY,
            ArmConstants.WRIST_MAX_ACCELERATION);

        // Initialize the ProfiledPIDControllers using your PID gains and constraints.
        shoulderController = new ProfiledPIDController(
            ArmConstants.kShoulderP,
            ArmConstants.kShoulderI,
            ArmConstants.kShoulderD,
            shoulderConstraints);
        wristController    = new ProfiledPIDController(
            ArmConstants.kWristP,
            ArmConstants.kWristI,
            ArmConstants.kWristD,
            wristConstraints);

        // Initialize setpoints to the current positions.
        // shoulderSetpoint = shoulderEncoder.getPosition();
        // wristSetpoint = wristEncoder.getPosition();
        // shoulderController.setGoal(shoulderSetpoint);
        // wristController.setGoal(wristSetpoint);
    }

    /**
     * Moves the shoulder to the desired angle (in degrees).
     * Conversion: rotations = degrees / (360 * gear ratio)
     */
    public void moveShoulderToSetpoint(double setpoint) {
        shoulderSetpoint = setpoint;
        double error     = setpoint - getShoulderAngle();
        double PIDoutput = error * ArmConstants.kShoulderP;
        PIDoutput = Math.min(Math.abs(PIDoutput), ArmConstants.MAX_SHOULDER_UP_SPEED) * Math.signum(PIDoutput);
        setShoulderSpeed(PIDoutput);
    }

    public void setShoulderSpeed(double speed) {
        shoulderMotor.set(speed);
    }

    /**
     * Moves the wrist to the desired angle (in degrees).
     */
    public void moveWristToSetpoint(double setpoint) {
        wristSetpoint = setpoint;
        double error     = setpoint - getWristEncoderPosition();
        double PIDoutput = error * ArmConstants.kWristP;
        PIDoutput = Math.min(Math.abs(PIDoutput), ArmConstants.MAX_WRIST_SPEED) * Math.signum(PIDoutput);
        setWristSpeed(PIDoutput);
    }

    public boolean hasGamePiece() {
        // Assuming active-low signal (true when object is detected)
        return !intakeSensor.get();
    }

    public void isGroundLock(Boolean groundLock) {
        isGroundLock = groundLock;
    }

    /*
     * Set intake to desired speed
     * if isReversed, wheels will intake coral
     * if !isReversed, wheels will release coral
     * The intake will stop once a game piece has been aquired
     */
    public void setIntakeSpeed(double intakeSpeed, boolean isReversed) {
        intakeMotor.set(VictorSPXControlMode.PercentOutput, isReversed ? -intakeSpeed : intakeSpeed);
        System.out.println("Intaking...");
    }

    /*
     * Stop the wrist motor
     */
    public void stopWrist() {
        wristMotor.stopMotor();
    }

    public void setWristSpeed(double speed) {
        wristMotor.set(speed);
    }

    /*
     * Stop all motors
     */
    public void stop() {
        shoulderMotor.stopMotor();
        stopWrist();
    }

    /*
     * Returns shoulder encoder position
     * If multiple rotations have occured, values will be added to total
     */
    public double getShoulderEncoderPosition() {
        return shoulderEncoder.getPosition();
    }

    public double getShoulderAngle() {
        // return the position as an angle
        return ((shoulderAbsoluteEncoder.getPosition() / ArmConstants.SHOULDER_GEAR_RATIO) * 360) - ArmConstants.SHOULDER_OFFSET;
    }

    /*
     * Returns wrist encoder position
     * If multiple rotations have occured, values will be added to total
     */
    public double getWristEncoderPosition() {
        return wristEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // Shoulder control: calculate output voltage from the ProfiledPIDController
        // double shoulderMeasurement = shoulderEncoder.getPosition();
        // double shoulderOutput = shoulderController.calculate(shoulderMeasurement);
        // double shoulderFollowerOutput = shoulderFollower.getAppliedOutput();
        // shoulderMotor.setVoltage(shoulderOutput);

        // // Wrist control: calculate output voltage from the ProfiledPIDController
        // double wristMeasurement = wristEncoder.getPosition();
        // double wristOutput = wristController.calculate(wristMeasurement);
        // wristMotor.setVoltage(wristOutput);

        // Update SmartDashboard
        SmartDashboard.putNumber("Arm/Shoulder Angle", getShoulderAngle());
        SmartDashboard.putBoolean("Intake/Game Piece Detected", hasGamePiece());
        SmartDashboard.putBoolean("Arm/GroundLocked", isGroundLock);
        // SmartDashboard.putNumber("Arm/Shoulder Motor Output", shoulderOutput);
        // SmartDashboard.putNumber("Arm/Shoulder Follower Output", shoulderFollowerOutput);
        // SmartDashboard.putNumber("Arm/Shoulder Position", shoulderMeasurement);
        SmartDashboard.putNumber("Arm/Shoulder Setpoint", shoulderSetpoint);
        SmartDashboard.putNumber("Arm/Wrist Position", getWristEncoderPosition());
        SmartDashboard.putNumber("Arm/Wrist Setpoint", wristSetpoint);
    }
}
