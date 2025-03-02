package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax                  lowerStageMotor;
    private final SparkMax                  upperStageMotor;

    private final RelativeEncoder           lowerStageEncoder;
    private final RelativeEncoder           upperStageEncoder;

    private final SparkClosedLoopController lowerStageClosedLoop;
    private final SparkClosedLoopController upperStageClosedLoop;

    private double                          upperStageSetpoint;

    public ElevatorSubsystem() {
        // Initialize motors
        lowerStageMotor      = new SparkMax(ElevatorConstants.LOWER_STAGE_MOTOR_CANID, MotorType.kBrushless);
        upperStageMotor      = new SparkMax(ElevatorConstants.UPPER_STAGE_MOTOR_CANID, MotorType.kBrushless);

        // Retrieve controllers and encoders
        lowerStageClosedLoop = lowerStageMotor.getClosedLoopController();
        lowerStageEncoder    = lowerStageMotor.getEncoder();
        upperStageClosedLoop = upperStageMotor.getClosedLoopController();
        upperStageEncoder    = upperStageMotor.getEncoder();

        // Configure lower stage motor (position-based control)
        SparkMaxConfig lowerStageConfig = new SparkMaxConfig();
        lowerStageConfig.closedLoop
            .p(ElevatorConstants.kLowerStageP)
            .i(ElevatorConstants.kLowerStageI)
            .d(ElevatorConstants.kLowerStageD)
            .outputRange(ElevatorConstants.LOWER_STAGE_MIN_OUTPUT, ElevatorConstants.LOWER_STAGE_MAX_OUTPUT);

        // Configure upper stage motor (motion control)
        SparkMaxConfig upperStageConfig = new SparkMaxConfig();
        upperStageConfig.closedLoop
            .p(ElevatorConstants.kUpperStageP)
            .i(ElevatorConstants.kUpperStageI)
            .d(ElevatorConstants.kUpperStageD)
            .outputRange(ElevatorConstants.UPPER_STAGE_MIN_OUTPUT, ElevatorConstants.UPPER_STAGE_MAX_OUTPUT);
        upperStageConfig.closedLoop.maxMotion
            .maxVelocity(ElevatorConstants.UPPER_STAGE_MAX_VELOCITY)
            .maxAcceleration(ElevatorConstants.UPPER_STAGE_MAX_ACCELERATION)
            .allowedClosedLoopError(ElevatorConstants.UPPER_STAGE_ALLOWED_CLOSED_LOOP_ERROR);

        upperStageSetpoint = upperStageEncoder.getPosition();
    }

    public void moveLowerStageUp() {
        lowerStageClosedLoop.setReference(ElevatorConstants.LOWER_STAGE_MAX_HEIGHT, ControlType.kPosition);
    }

    public void moveLowerStageDown() {
        lowerStageClosedLoop.setReference(ElevatorConstants.LOWER_STAGE_MIN_HEIGHT, ControlType.kPosition);
    }

    public void stopLowerStage() {
        lowerStageMotor.stopMotor();
    }

    public void setUpperStageHeight(double heightRotations) {
        upperStageSetpoint = heightRotations;
        upperStageClosedLoop.setReference(heightRotations, ControlType.kMAXMotionPositionControl);
    }

    public double feetToRotations(double heightFeet) {
        return heightFeet * ElevatorConstants.ROTATIONS_TO_FEET;
    }

    public void setUpperStagePosition(int setpointIndex) {
        if (setpointIndex < 0 || setpointIndex >= ElevatorConstants.ELEVATOR_UPPER_STAGE_SETPOINTS.length) {
            SmartDashboard.putString("Elevator Error", "Invalid setpoint index: " + setpointIndex);
            return;
        }
        upperStageSetpoint = ElevatorConstants.ELEVATOR_UPPER_STAGE_SETPOINTS[setpointIndex];
        upperStageClosedLoop.setReference(upperStageSetpoint, ControlType.kMAXMotionPositionControl);
    }

    public void stopUpperStage() {
        upperStageMotor.stopMotor();
    }

    public void resetUpperStageEncoder() {
        upperStageEncoder.setPosition(0.0);
    }

    public double getUpperStageEncoderPosition() {
        return upperStageEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/Lower Stage Position", lowerStageEncoder.getPosition());
        SmartDashboard.putNumber("Elevator/Upper Stage Position", upperStageEncoder.getPosition());
        SmartDashboard.putNumber("Elevator/Upper Stage Setpoint", upperStageSetpoint);
    }
}
