package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    // Motors for the two stages of the elevator
    private final MotorController          lowerStageMotor;
    private final SparkMax                 upperStageMotor;
    private final SparkMaxAlternateEncoder upperStageEncoder;
    private final PIDController            topPIDController;

    // Conversion factors: Encoder Rotations → Feet
    private static final double            DRUM_DIAMETER_INCHES  = ElevatorConstants.ELEVATOR_DRUM_DIAMETER_INCHES;
    private static final double            GEAR_RATIO            = ElevatorConstants.ELEVATOR_GEAR_RATIO;
    private static final double            INCHES_TO_FEET        = Constants.INCHES_TO_FEET;
    private static final double            ROTATIONS_TO_FEET     = (Math.PI * DRUM_DIAMETER_INCHES * INCHES_TO_FEET) / GEAR_RATIO;

    // Predefined setpoints for the upper stage (in feet)
    public static final double[]           UPPER_STAGE_SETPOINTS = ElevatorConstants.ELEVATOR_UPPER_STAGE_SETPOINTS;

    // PID Constants for the upper stage
    private static final double            kP                    = ElevatorConstants.kElevatrorP;
    private static final double            kI                    = ElevatorConstants.kElevatrorI;
    private static final double            kD                    = ElevatorConstants.kElevatrorD;

    /**
     * Constructs the ElevatorSubsystem.
     *
     * @param lowerMotorPort Port for the lower stage motor.
     * @param upperMotorPort Port for the upper stage motor.
     */
    public ElevatorSubsystem(int lowerMotorPort, int upperMotorPort) {
        lowerStageMotor   = new SparkMax(lowerMotorPort, MotorType.kBrushed);
        upperStageMotor   = new SparkMax(upperMotorPort, MotorType.kBrushless);

        upperStageEncoder = (SparkMaxAlternateEncoder) upperStageMotor.getEncoder();

        topPIDController  = new PIDController(kP, kI, kD);
        topPIDController.setTolerance(0.01); // Tolerance for precise stopping
    }

    /**
     * Converts a distance in feet to the equivalent number of encoder rotations.
     *
     * @param feet the distance in feet.
     * @return the equivalent encoder rotations.
     */
    private double feetToRotations(double feet) {
        return feet / ROTATIONS_TO_FEET;
    }

    /**
     * Moves the lower stage upward.
     */
    public void moveLowerStageUp() {
        lowerStageMotor.set(ElevatorConstants.ELEVATOR_BOTTOM_STAGE_RISE_SPEED);
    }

    /**
     * Moves the lower stage downward.
     */
    public void moveLowerStageDown() {
        lowerStageMotor.set(ElevatorConstants.ELEVATOR_BOTTOM_STAGE_FALL_SPEED);
    }

    /**
     * Stops the lower stage motor.
     */
    public void stopLowerStage() {
        lowerStageMotor.set(0.0);
    }

    /**
     * Moves the upper stage to a specific setpoint based on the provided index.
     *
     * @param setpointIndex the index of the desired setpoint.
     */
    public void setUpperStagePosition(int setpointIndex) {
        if (setpointIndex < 0 || setpointIndex >= UPPER_STAGE_SETPOINTS.length) {
            SmartDashboard.putString("Elevator Error", "Invalid setpoint index: " + setpointIndex);
            return;
        }
        double targetFeet       = UPPER_STAGE_SETPOINTS[setpointIndex];
        double targetRotations  = feetToRotations(targetFeet);
        double currentRotations = upperStageEncoder.getPosition();
        double pidOutput        = topPIDController.calculate(currentRotations, targetRotations);
        upperStageMotor.set(pidOutput);
    }

    /**
     * Stops the upper stage motor.
     */
    public void stopUpperStage() {
        upperStageMotor.set(0.0);
    }

    /**
     * Resets the upper stage encoder to zero.
     */
    public void resetUpperStageEncoder() {
        upperStageEncoder.setPosition(0.0);
    }

    public double getUpperStageEncoderPosition() {
        return upperStageEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // Update SmartDashboard with current diagnostics
        SmartDashboard.putNumber("Upper Stage Encoder", upperStageEncoder.getPosition());
        SmartDashboard.putNumber("Elevator PID Error", topPIDController.getPositionError());
    }
}
