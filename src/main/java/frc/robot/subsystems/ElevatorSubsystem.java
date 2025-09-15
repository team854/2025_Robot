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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax        lowerStageMotor;
    private final SparkMax        upperStageMotor;

    private final RelativeEncoder lowerStageEncoder;
    private final RelativeEncoder upperStageEncoder;
    private final AbsoluteEncoder upperStageAbsoluteEncoder;

    public ElevatorSubsystem() {
        // Initialize motors
        lowerStageMotor           = new SparkMax(ElevatorConstants.LOWER_STAGE_MOTOR_CANID, MotorType.kBrushless);
        upperStageMotor           = new SparkMax(ElevatorConstants.UPPER_STAGE_MOTOR_CANID, MotorType.kBrushless);

        // Retrieve encoders
        lowerStageEncoder         = lowerStageMotor.getEncoder();
        upperStageEncoder         = upperStageMotor.getEncoder();
        upperStageAbsoluteEncoder = upperStageMotor.getAbsoluteEncoder();

        // Create elevator soft limits
        SoftLimitConfig upperStageSoftLimitConfig = new SoftLimitConfig();
        SoftLimitConfig lowerStageSoftLimitConfig = new SoftLimitConfig();

        // Create and apply lower SparkMax settings
        SparkMaxConfig  lowerStageConfig          = new SparkMaxConfig();
        lowerStageConfig.idleMode(IdleMode.kBrake);
        lowerStageConfig.inverted(true);
        lowerStageSoftLimitConfig.forwardSoftLimitEnabled(false);
        lowerStageSoftLimitConfig.forwardSoftLimit(ElevatorConstants.ELEVATOR_UPPER_STAGE_UPPER_LIMIT);
        lowerStageSoftLimitConfig.reverseSoftLimitEnabled(false);
        lowerStageSoftLimitConfig.reverseSoftLimit(ElevatorConstants.ELEVATOR_UPPER_STAGE_LOWER_LIMIT);
        lowerStageConfig.apply(lowerStageConfig);
        lowerStageMotor.configure(lowerStageConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Create and apply upper SparkMax settings
        SparkMaxConfig upperStageConfig = new SparkMaxConfig();
        upperStageConfig.idleMode(IdleMode.kBrake);
        upperStageConfig.inverted(true);
        upperStageConfig.absoluteEncoder.inverted(true);
        upperStageConfig.absoluteEncoder.zeroOffset(ElevatorConstants.ELEVATOR_TOP_STAGE_ENCODER_ZERO_OFFSET);
        upperStageSoftLimitConfig.forwardSoftLimitEnabled(false);
        upperStageSoftLimitConfig.forwardSoftLimit(ElevatorConstants.ELEVATOR_LOWER_STAGE_UPPER_LIMIT);
        upperStageSoftLimitConfig.reverseSoftLimitEnabled(false);
        upperStageSoftLimitConfig.reverseSoftLimit(ElevatorConstants.ELEVATOR_LOWER_STAGE_LOWER_LIMIT);
        upperStageConfig.apply(upperStageSoftLimitConfig);
        upperStageMotor.configure(upperStageConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        upperStageEncoder.setPosition(getUpperStageAbsoluteEncoderPosition());
    }

    public void stopLowerStage() {
        lowerStageMotor.stopMotor();
        System.out.println("Stopping lower stage");
    }

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

    public void resetUpperStageEncoder() {
        upperStageEncoder.setPosition(0.0);
    }

    public void resetLowerStageEncoder() {
        lowerStageEncoder.setPosition(0.0);
    }

    public double getUpperStageEncoderPosition() {
        return upperStageEncoder.getPosition();
    }

    public double getUpperStageAbsoluteEncoderPosition() {
        return upperStageAbsoluteEncoder.getPosition();
    }

    public double getLowerStageEncoderPosition() {
        return lowerStageEncoder.getPosition();
    }

    public double getUpperStageSpeed() {
        return upperStageMotor.get();
    }

    public double getLowerStageSpeed() {
        return lowerStageMotor.get();
    }

    /*
     * It is very important that these functions remain functional
     * If these are not working properly or if the elevator is hitting the hard stops too hard, adjust the corresponding constant
     */
    public boolean isLowerStageAtLowerLimit() {
        return getLowerStageEncoderPosition() < ElevatorConstants.ELEVATOR_LOWER_STAGE_LOWER_LIMIT;
    }

    public boolean isLowerStageAtUpperLimit() {
        return getLowerStageEncoderPosition() > ElevatorConstants.ELEVATOR_LOWER_STAGE_UPPER_LIMIT;
    }

    public boolean isUpperStageAtLowerLimit() {
        return getUpperStageEncoderPosition() < ElevatorConstants.ELEVATOR_UPPER_STAGE_LOWER_LIMIT;
    }

    public boolean isUpperStageAtUpperLimit() {
        return getUpperStageEncoderPosition() > ElevatorConstants.ELEVATOR_UPPER_STAGE_UPPER_LIMIT;
    }

    @Override
    public void periodic() {

        // Update SmartDashboard
        SmartDashboard.putNumber("Elevator/Lower Stage Encoder", getLowerStageEncoderPosition());
        SmartDashboard.putNumber("Elevator/Upper Stage Encoder", getUpperStageEncoderPosition());
        SmartDashboard.putNumber("Elevator/Upper Stage Absolute Encoder", getUpperStageAbsoluteEncoderPosition());
    }
}
