package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax              lowerStageMotor;
    private final SparkMax              upperStageMotor;

    private final RelativeEncoder       lowerStageEncoder;
    private final RelativeEncoder       upperStageEncoder;
    private final AbsoluteEncoder       upperStageAbsoluteEncoder;

    private final ProfiledPIDController lowerStageController;
    private final ProfiledPIDController upperStageController;

    public ElevatorSubsystem() {
        // Initialize motors
        lowerStageMotor           = new SparkMax(ElevatorConstants.LOWER_STAGE_MOTOR_CANID, MotorType.kBrushless);
        upperStageMotor           = new SparkMax(ElevatorConstants.UPPER_STAGE_MOTOR_CANID, MotorType.kBrushless);

        // Retrieve encoders
        lowerStageEncoder         = lowerStageMotor.getEncoder();
        upperStageEncoder         = upperStageMotor.getEncoder();
        upperStageAbsoluteEncoder = upperStageMotor.getAbsoluteEncoder();

        // Define trapezoidal motion profile constraints
        TrapezoidProfile.Constraints lowerStageConstraints = new TrapezoidProfile.Constraints(
            ElevatorConstants.LOWER_STAGE_MAX_VELOCITY,
            ElevatorConstants.LOWER_STAGE_MAX_ACCELERATION);

        TrapezoidProfile.Constraints upperStageConstraints = new TrapezoidProfile.Constraints(
            ElevatorConstants.UPPER_STAGE_MAX_VELOCITY,
            ElevatorConstants.UPPER_STAGE_MAX_ACCELERATION);

        /*
         * Create SparkMAX configs and burn them to the motors
         * Both motors should be set to brake to minimize how fast the elevator slides when disabled
         */
        SparkMaxConfig               lowerStageConfig      = new SparkMaxConfig();
        lowerStageConfig.idleMode(IdleMode.kBrake);
        lowerStageConfig.inverted(false); // FIX ME!!!!
        lowerStageMotor.configure(lowerStageConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig upperStageConfig = new SparkMaxConfig();
        upperStageConfig.idleMode(IdleMode.kBrake);
        upperStageConfig.inverted(false); // FIX ME!!!!!!!!
        upperStageConfig.absoluteEncoder.inverted(false); // FIX ME AS WELL!!!
        upperStageConfig.absoluteEncoder.zeroOffset(ElevatorConstants.ELEVATOR_TOP_STAGE_ENCODER_ZERO_OFFSET);
        upperStageMotor.configure(upperStageConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        /*
         * Initialize soft limits for elevator so that it will not go past a certain point
         */
        SoftLimitConfig upperStageSoftLimitConfig = new SoftLimitConfig();
        SoftLimitConfig lowerStageSoftLimitConfig = new SoftLimitConfig();

        upperStageSoftLimitConfig.forwardSoftLimitEnabled(true);
        upperStageSoftLimitConfig.forwardSoftLimit(ElevatorConstants.ELEVATOR_LOWER_STAGE_UPPER_LIMIT);
        upperStageSoftLimitConfig.reverseSoftLimitEnabled(true);
        upperStageSoftLimitConfig.forwardSoftLimit(ElevatorConstants.ELEVATOR_LOWER_STAGE_LOWER_LIMIT);

        lowerStageSoftLimitConfig.forwardSoftLimitEnabled(true);
        lowerStageSoftLimitConfig.forwardSoftLimit(ElevatorConstants.ELEVATOR_UPPER_STAGE_UPPER_LIMIT);
        lowerStageSoftLimitConfig.reverseSoftLimitEnabled(true);
        lowerStageSoftLimitConfig.forwardSoftLimit(ElevatorConstants.ELEVATOR_UPPER_STAGE_LOWER_LIMIT);

        // Initialize profiled PID controllers
        lowerStageController = new ProfiledPIDController(
            ElevatorConstants.kLowerStageP,
            ElevatorConstants.kLowerStageI,
            ElevatorConstants.kLowerStageD,
            lowerStageConstraints);

        upperStageController = new ProfiledPIDController(
            ElevatorConstants.kUpperStageP,
            ElevatorConstants.kUpperStageI,
            ElevatorConstants.kUpperStageD,
            upperStageConstraints);

        /*
         * Set relative encoder offsets
         */
        upperStageEncoder.setPosition(getUpperStageAbsoluteEncoderPosition());
    }

    /*
     * Takes in a target height in feet, converts to rotations
     * Sets the elevator height using a trapezoid profile
     * Setting Lower Stage
     */
    public void setLowerStage(double height) {
        lowerStageController.setGoal(height);
        System.out.println("Setting lower stage to: " + height);

    }

    // Stops lower stage
    public void stopLowerStage() {
        lowerStageMotor.stopMotor();
        System.out.println("Stopping lower stage");
    }

    /*
     * Takes in a target height in feet, converts to rotations
     * Sets the elevator height using a trapezoid profile
     * Setting Upper Stage
     */
    public void setUpperStage(double height) {
        upperStageController.setGoal(height);
        System.out.println("Setting upper stage to: " + height);
    }

    // Stops upper stage
    public void stopUpperStage() {
        upperStageMotor.stopMotor();
        System.out.println("Stopping upper stage");
    }

    public void setTopStageSpeed(double speed) {
        upperStageMotor.set(speed);
    }

    public void setBottomStageSpeed(double speed) {
        lowerStageMotor.set(speed);
    }

    /*
     * Conversion factor which converts a height in feet into expected encoder rotations
     */
    public double upperFeetToRotations(double heightFeet) {
        return heightFeet / ElevatorConstants.ROTATIONS_TO_FEET_UPPER;
    }

    public double lowerFeetToRotations(double heightFeet) {
        return heightFeet / ElevatorConstants.ROTATIONS_TO_FEET_LOWER;
    }

    /*
     * Zeros the upper stage encoder position
     */
    public void resetUpperStageEncoder() {
        upperStageEncoder.setPosition(0.0);
    }

    /*
     * Zeros the lower stage encoder position
     */
    public void resetLowerStageEncoder() {
        lowerStageEncoder.setPosition(0.0);
    }

    /*
     * Returns the value of the upper stage encoder in degrees
     * If the motor completes multiple rotations, the value of all rotations will be added into this value
     */
    public double getUpperStageEncoderPosition() {
        return upperStageEncoder.getPosition();
    }

    /*
     * Returns upper stage absolute encoder position
     * use for setting initial relative encoder position
     */
    public double getUpperStageAbsoluteEncoderPosition() {
        return upperStageAbsoluteEncoder.getPosition();
    }

    /*
     * Returns the height of the upper stage in feet
     */
    public double getUpperStageHeight() {
        return getUpperStageEncoderPosition() * ElevatorConstants.ROTATIONS_TO_FEET_UPPER;
    }

    /*
     * Returns the value of the lower stage encoder in degrees
     * If the motor completes multiple rotations, the value of all rotations will be added into this value
     */
    public double getLowerStageEncoderPosition() {
        return lowerStageEncoder.getPosition();
    }

    /*
     * Returns the height of the lower stage in feet
     */
    public double getLowerStageHeight() {
        return getLowerStageEncoderPosition() * ElevatorConstants.ROTATIONS_TO_FEET_LOWER;
    }

    /*
     * Returns upper stage speed
     */
    public double getUpperStageSpeed() {
        return upperStageMotor.get();
    }

    /*
     * Returns lower stage speed
     */
    public double getLowerStageSpeed() {
        return lowerStageMotor.get();
    }

    /*
     * Sets motor speeds to zero if limit is exceeded
     */
    public void checkElevatorLimits(double upperStageSpeed, double lowerStageSpeed) {
        if (getUpperStageEncoderPosition() <= ElevatorConstants.ELEVATOR_UPPER_STAGE_LOWER_LIMIT && upperStageSpeed < 0) {
            upperStageMotor.set(0);
            System.out.println("Upper stage at lower limit");
        }
        if (getUpperStageEncoderPosition() >= ElevatorConstants.ELEVATOR_UPPER_STAGE_UPPER_LIMIT && upperStageSpeed > 0) {
            upperStageMotor.set(0);
            System.out.println("Upper stage at upper limit");
        }
        if (getLowerStageEncoderPosition() >= ElevatorConstants.ELEVATOR_LOWER_STAGE_LOWER_LIMIT && lowerStageSpeed > 0) {
            lowerStageMotor.set(0);
            System.out.println("Lower stage at lower limit");
        }
        if (getLowerStageEncoderPosition() >= ElevatorConstants.ELEVATOR_LOWER_STAGE_UPPER_LIMIT && lowerStageSpeed > 0) {
            lowerStageMotor.set(0);
            System.out.println("Lower stage at upper limit");
        }
    }

    @Override
    public void periodic() {

        checkElevatorLimits(getUpperStageSpeed(), getLowerStageSpeed());
        // Lower stage control
        // double lowerStageMeasurement = lowerStageEncoder.getPosition();
        // double lowerStageOutput = lowerStageController.calculate(lowerStageMeasurement);
        // lowerStageMotor.setVoltage(lowerStageOutput);

        // // Upper stage control
        // double upperStageMeasurement = upperStageEncoder.getPosition();
        // double upperStageOutput = upperStageController.calculate(upperStageMeasurement);
        // upperStageMotor.setVoltage(upperStageOutput);

        // Update SmartDashboard
        SmartDashboard.putNumber("Elevator/Lower Stage Encoder", getLowerStageEncoderPosition());
        SmartDashboard.putNumber("Elevator/Upper Stage Encoder", getUpperStageEncoderPosition());
        SmartDashboard.putNumber("Elevator/Lower Stage Position", getLowerStageHeight());
        SmartDashboard.putNumber("Elevator/Upper Stage Position", getUpperStageHeight());
        SmartDashboard.putNumber("Elevator/Lower Stage Setpoint", lowerStageController.getSetpoint().position);
        SmartDashboard.putNumber("Elevator/Upper Stage Setpoint", upperStageController.getSetpoint().position);

    }
}
