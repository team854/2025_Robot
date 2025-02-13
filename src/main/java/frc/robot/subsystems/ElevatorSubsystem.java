package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private final MotorController          bottomMotor;
    private final SparkMax                 topMotor;
    private final SparkMaxAlternateEncoder topEncoder;
    private final PIDController            pidController;

    // Conversion factor: Encoder Rotations → Feet
    private static final double            DRUM_DIAMETER_INCHES = 2.0;
    private static final double            GEAR_RATIO           = 5.0;

    private static final double            INCHES_TO_FEET       = 1.0 / 12.0;
    private static final double            ROTATIONS_TO_FEET    = (Math.PI * DRUM_DIAMETER_INCHES * INCHES_TO_FEET) / GEAR_RATIO;

    // Define setpoints for the second stage
    private static final double[]          SETPOINTS_FEET       = { 0.0, 1.0, 2.0, 3.0, 4.0 };

    // PID Constants
    private static final double            kP                   = 1.0;
    private static final double            kI                   = 0.0;
    private static final double            kD                   = 0.0;

    public ElevatorSubsystem(int bottomMotorPort, int topMotorPort) {
        bottomMotor   = new SparkMax(bottomMotorPort, MotorType.kBrushed);
        topMotor      = new SparkMax(topMotorPort, MotorType.kBrushless);

        topEncoder    = (SparkMaxAlternateEncoder) topMotor.getEncoder();

        // Configure PID controller for second stage
        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(0.01); // Small tolerance for precise stopping
    }

    /** Moves the bottom stage up */
    public void moveBottomUp() {
        bottomMotor.set(1.0); // Full power up
    }

    /** Moves the bottom stage down */
    public void moveBottomDown() {
        bottomMotor.set(-1.0); // Full power down
    }

    /** Stops the bottom stage */
    public void stopBottom() {
        bottomMotor.set(0.0);
    }

    /** Moves the second stage to a specific setpoint in feet */
    public void setTopStagePosition(int positionIndex) {
        if (positionIndex < 0 || positionIndex >= SETPOINTS_FEET.length) {
            System.out.println("Invalid setpoint index");
            return;
        }

        double targetFeet      = SETPOINTS_FEET[positionIndex];
        double targetRotations = targetFeet / ROTATIONS_TO_FEET;

        double power           = pidController.calculate(topEncoder.getPosition(), targetRotations);
        topMotor.set(power);
    }

    /** Stops the second stage */
    public void stopTop() {
        topMotor.set(0.0);
    }

    @Override
    public void periodic() {
        // Display encoder value on SmartDashboard (if needed)
        System.out.println("Top Encoder Position: " + topEncoder.getPosition());
    }
}
