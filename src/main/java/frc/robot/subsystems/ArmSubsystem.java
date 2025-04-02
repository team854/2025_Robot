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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Tolerances;

public class ArmSubsystem extends SubsystemBase {

    // Motor controllers for shoulder and wrist
    private final SparkMax        shoulderMotor;
    private final SparkMax        wristMotor;

    // Controller for the intake wheels
    private final VictorSPX       intakeMotor;

    // Intake Sensor
    private final DigitalInput    intakeSensor;

    // Encoders
    private final RelativeEncoder shoulderEncoder;
    private final RelativeEncoder wristEncoder;
    private final AbsoluteEncoder shoulderAbsoluteEncoder;

    // Desired setpoints (in appropriate units)
    private double                shoulderTargetSetpoint; // in degrees
    private double                wristSetpoint;

    // WPILib PID controller for the shoulder
    private final PIDController   shoulderPID;

    public ArmSubsystem() {
        // Initialize motors
        shoulderMotor           = new SparkMax(ArmConstants.SHOULDER_MOTOR_ID, MotorType.kBrushless);
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
        SparkMaxConfig shoulderConfig = new SparkMaxConfig();
        shoulderConfig.idleMode(IdleMode.kBrake);
        shoulderConfig.inverted(false);
        shoulderConfig.absoluteEncoder.zeroOffset(ArmConstants.SHOULDER_ABSOLUTE_ENCODER_ZERO_OFFSET);
        shoulderConfig.absoluteEncoder.inverted(false);
        shoulderMotor.configure(shoulderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        /*
         * Wrist Config
         */
        SparkMaxConfig wristConfig = new SparkMaxConfig();
        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize the shoulder PID controller with gains from ArmConstants
        shoulderPID = new PIDController(ArmConstants.kShoulderP, ArmConstants.kShoulderI, ArmConstants.kShoulderD);
        // Set tolerance in degrees. This value is defined in your ArmConstants.
        shoulderPID.setTolerance(Tolerances.SHOULDER_TOLERANCE);

        // Initialize setpoint to the current angle to avoid an initial jump.
        shoulderTargetSetpoint = getShoulderAngle();
    }

    /**
     * Sets the desired shoulder angle (in degrees) as the target for the PID controller.
     */
    public void setShoulderSetpoint(double setpoint) {
        shoulderTargetSetpoint = setpoint;
    }

    /**
     * Returns true if the shoulder is within the defined tolerance of the target setpoint.
     */
    public boolean isShoulderAtSetpoint() {
        // atSetpoint() returns true if the error is within the tolerance specified via setTolerance.
        return shoulderPID.atSetpoint();
    }

    /**
     * Updates the shoulder PID control loop.
     * This should be called in periodic() so that the PID loop continuously updates.
     */
    private void updateShoulderPID() {
        double currentAngle = getShoulderAngle();
        double pidOutput    = shoulderPID.calculate(currentAngle, shoulderTargetSetpoint);
        // If within tolerance, we want to stop the motor.
        if (shoulderPID.atSetpoint()) {
            pidOutput = 0.0;
        }
        // Apply the PID output to the shoulder motor.
        setShoulderSpeed(pidOutput);
    }

    public void setShoulderSpeed(double speed) {
        // The offset of 0.045 can be adjusted or removed if not needed.
        shoulderMotor.set(speed + 0.045);
    }

    /**
     * Moves the wrist to the desired angle (in degrees).
     */
    public void moveWristToSetpoint(double setpoint) {
        wristSetpoint = setpoint;
        System.out.println("Setting wrist to: " + setpoint + " degrees");
        // Implementation for wrist PID control can be added similarly if needed.
    }

    public boolean hasGamePiece() {
        // Assuming active-low signal (true when object is detected)
        return !intakeSensor.get();
    }

    /*
     * Set intake to desired speed.
     * If isReversed, wheels will intake the game piece.
     * If not reversed, wheels will release the game piece.
     */
    public void setIntakeSpeed(double intakeSpeed, boolean isReversed) {
        intakeMotor.set(VictorSPXControlMode.PercentOutput, isReversed ? -intakeSpeed : intakeSpeed);
        System.out.println("Intaking...");
    }

    /*
     * Stop the wrist motor.
     */
    public void stopWrist() {
        wristMotor.stopMotor();
    }

    public void setWristSpeed(double speed) {
        wristMotor.set(speed);
    }

    /*
     * Stop all motors.
     */
    public void stop() {
        shoulderMotor.stopMotor();
        stopWrist();
    }

    /*
     * Returns the shoulder encoder position (in rotations).
     */
    public double getShoulderEncoderPosition() {
        return shoulderEncoder.getPosition();
    }

    /**
     * Returns the current shoulder angle in degrees.
     * Calculation: (absolute encoder position / gear ratio) * 360, then adjusted by an offset.
     */
    public double getShoulderAngle() {
        return ((shoulderAbsoluteEncoder.getPosition() / ArmConstants.SHOULDER_GEAR_RATIO) * 360) - ArmConstants.SHOULDER_OFFSET;
    }

    /*
     * Returns the wrist encoder position.
     */
    public double getWristEncoderPosition() {
        return wristEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // Update the shoulder PID control loop.
        updateShoulderPID();

        // Update SmartDashboard with current measurements and setpoints.
        SmartDashboard.putNumber("Arm/Shoulder Angle", getShoulderAngle());
        SmartDashboard.putBoolean("Arm/Shoulder At Setpoint", isShoulderAtSetpoint());
        SmartDashboard.putNumber("Arm/Shoulder Setpoint", shoulderTargetSetpoint);
        SmartDashboard.putNumber("Arm/Wrist Setpoint", wristSetpoint);
        SmartDashboard.putBoolean("Intake/Game Piece Detected", hasGamePiece());
    }
}
