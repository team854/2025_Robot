package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Tolerances;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax        lowerStageMotor;
    private final SparkMax        upperStageMotor;

    private final RelativeEncoder lowerStageEncoder;
    private final RelativeEncoder upperStageEncoder;
    private final AbsoluteEncoder upperStageAbsoluteEncoder;

    // WPILib PID controllers for the lower and upper stages
    private final PIDController   lowerPID;
    private final PIDController   upperPID;

    // Setpoints for PID control (in encoder rotations)
    private double                lowerSetpoint = 0.0;
    private double                upperSetpoint = 0.0;

    public ElevatorSubsystem() {
        // Initialize motors
        lowerStageMotor           = new SparkMax(ElevatorConstants.LOWER_STAGE_MOTOR_CANID, MotorType.kBrushless);
        upperStageMotor           = new SparkMax(ElevatorConstants.UPPER_STAGE_MOTOR_CANID, MotorType.kBrushless);

        // Retrieve encoders
        lowerStageEncoder         = lowerStageMotor.getEncoder();
        upperStageEncoder         = upperStageMotor.getEncoder();
        upperStageAbsoluteEncoder = upperStageMotor.getAbsoluteEncoder();

        /*
         * Create SparkMAX configs and burn them to the motors.
         * Both motors should be set to brake to minimize how fast the elevator slides when disabled.
         */
        SparkMaxConfig upperStageConfig = new SparkMaxConfig();
        SparkMaxConfig lowerStageConfig = new SparkMaxConfig();

        /*
         * Lower stage motor config
         */
        lowerStageConfig.idleMode(IdleMode.kBrake);
        lowerStageConfig.inverted(true);

        /*
         * Upper stage motor config
         */

        upperStageConfig.idleMode(IdleMode.kBrake);
        upperStageConfig.inverted(true);

        /*
         * Upper stage absolute encoder
         */
        upperStageConfig.absoluteEncoder.inverted(true);
        upperStageConfig.absoluteEncoder.zeroOffset(ElevatorConstants.ELEVATOR_TOP_STAGE_ENCODER_ZERO_OFFSET);

        /*
         * Configure both sparkMAX controllers with the configs created
         */
        upperStageMotor.configure(upperStageConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lowerStageMotor.configure(lowerStageConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        /*
         * Initialize WPILIB PIDControllers
         * These use WPILIB's PIDController class and will run on the RoboRIO
         */
        lowerPID = new PIDController(ElevatorConstants.kLowerStageP, ElevatorConstants.kLowerStageI,
            ElevatorConstants.kLowerStageD);
        upperPID = new PIDController(ElevatorConstants.kUpperStageP, ElevatorConstants.kUpperStageI,
            ElevatorConstants.kUpperStageD);

        /*
         * Set tolerances for the PID so we can later check if the elevator is in position
         */
        lowerPID.setTolerance(Tolerances.ELEVATOR_LOWER_TOLERANCE);
        upperPID.setTolerance(Tolerances.ELEVATOR_UPPER_TOLERANCE);
    }

    /*
     * Sets the lower stage to the target height (in feet) using PID position control.
     */
    public void setLowerStage(double height) {
        System.out.println("Setting lower stage to: " + height + " feet");
        lowerSetpoint = lowerFeetToRotations(height);
    }

    /*
     * Stop lower stage motor
     */
    public void stopLowerStage() {
        lowerStageMotor.stopMotor();
        System.out.println("Stopping lower stage");
    }

    /*
     * Sets the upper stage to the target height (in feet) using PID position control.
     */
    public void setUpperStage(double height) {
        System.out.println("Setting upper stage to: " + height + " feet");
        upperSetpoint = upperFeetToRotations(height);
    }

    /*
     * Stops upper stage motor
     */
    public void stopUpperStage() {
        upperStageMotor.stopMotor();
        System.out.println("Stopping upper stage");
    }

    /*
     * Directly sets top stage motor speed
     */
    public void setTopStageSpeed(double speed) {
        upperStageMotor.set(speed);
    }

    /*
     * Directly sets bottom stage motor speed
     */
    public void setBottomStageSpeed(double speed) {
        lowerStageMotor.set(speed);
    }

    /*
     * Upper stage conversion
     * Conversion factor which converts a height in feet into expected encoder rotations
     */
    public double upperFeetToRotations(double heightFeet) {
        return heightFeet / ElevatorConstants.ROTATIONS_TO_FEET_UPPER;
    }

    /*
     * Lower stage conversion
     * Conversion factor which converts a height in feet into expected encoder rotations
     */
    public double lowerFeetToRotations(double heightFeet) {
        return heightFeet / ElevatorConstants.ROTATIONS_TO_FEET_LOWER;
    }

    /*
     * Zeros the upper stage encoder position.
     */
    public void resetUpperStageEncoder() {
        upperStageEncoder.setPosition(0.0);
    }

    /*
     * Zeros the lower stage encoder position.
     */
    public void resetLowerStageEncoder() {
        lowerStageEncoder.setPosition(0.0);
    }

    /*
     * Returns the value of the upper stage encoder in rotations.
     */
    public double getUpperStageEncoderPosition() {
        return upperStageEncoder.getPosition();
    }

    /*
     * Returns the upper stage absolute encoder position.
     * Use for setting initial relative encoder position.
     */
    public double getUpperStageAbsoluteEncoderPosition() {
        return upperStageAbsoluteEncoder.getPosition();
    }

    /*
     * Returns the height of the upper stage in feet.
     */
    public double getUpperStageHeight() {
        return getUpperStageEncoderPosition() * ElevatorConstants.ROTATIONS_TO_FEET_UPPER;
    }

    /*
     * Returns the value of the lower stage encoder in rotations.
     */
    public double getLowerStageEncoderPosition() {
        return lowerStageEncoder.getPosition();
    }

    /*
     * Returns the height of the lower stage in feet.
     */
    public double getLowerStageHeight() {
        return getLowerStageEncoderPosition() * ElevatorConstants.ROTATIONS_TO_FEET_LOWER;
    }

    /*
     * Returns upper stage speed.
     */
    public double getUpperStageSpeed() {
        return upperStageMotor.get();
    }

    /*
     * Returns lower stage speed.
     */
    public double getLowerStageSpeed() {
        return lowerStageMotor.get();
    }

    /*
     * Return true if the upper stage has reached its setpoint
     */
    public boolean isUpperAtSetpoint() {
        return upperPID.atSetpoint();
    }

    /*
     * Return true if the lower stage has reached its setpoint
     */
    public boolean isLowerAtSetpoint() {
        return upperPID.atSetpoint();
    }

    @Override
    public void periodic() {
        // Compute PID outputs based on current encoder positions and setpoints
        double lowerOutput = lowerPID.calculate(lowerStageEncoder.getPosition(), lowerSetpoint);
        double upperOutput = upperPID.calculate(upperStageEncoder.getPosition(), upperSetpoint);

        // Set motor outputs using PID controller outputs
        lowerStageMotor.set(lowerOutput);
        upperStageMotor.set(upperOutput);

        // Update SmartDashboard with encoder positions, setpoints, and PID outputs
        SmartDashboard.putNumber("Elevator/Lower Stage Encoder", getLowerStageEncoderPosition());
        SmartDashboard.putNumber("Elevator/Lower Stage Setpoint", lowerSetpoint);
        SmartDashboard.putNumber("Elevator/Lower Stage Output", lowerOutput);
        SmartDashboard.putNumber("Elevator/Upper Stage Encoder", getUpperStageEncoderPosition());
        SmartDashboard.putNumber("Elevator/Upper Stage Setpoint", upperSetpoint);
        SmartDashboard.putNumber("Elevator/Upper Stage Output", upperOutput);
        SmartDashboard.putNumber("Elevator/Upper Stage Absolute Encoder", getUpperStageAbsoluteEncoderPosition());
    }
}
