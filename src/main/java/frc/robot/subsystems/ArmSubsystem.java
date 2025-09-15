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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private final SparkMax        shoulderMotor;
    private final SparkMax        wristMotor;
    private final VictorSPX       intakeMotor;
    private final DigitalInput    intakeSensor;
    private final RelativeEncoder shoulderEncoder;
    private final RelativeEncoder wristEncoder;
    private final AbsoluteEncoder shoulderAbsoluteEncoder;
    private double                shoulderSetpoint;
    private double                wristSetpoint;
    private boolean               isGroundLock;

    public ArmSubsystem() {

        // Initialize motors and sensors
        shoulderMotor           = new SparkMax(ArmConstants.SHOULDER_MOTOR_ID, MotorType.kBrushless);
        wristMotor              = new SparkMax(ArmConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
        intakeMotor             = new VictorSPX(ArmConstants.INTAKE_MOTOR_ID);
        shoulderEncoder         = shoulderMotor.getEncoder();
        shoulderAbsoluteEncoder = shoulderMotor.getAbsoluteEncoder();
        wristEncoder            = wristMotor.getEncoder();
        intakeSensor            = new DigitalInput(ArmConstants.INTAKE_SENSOR_PORT);

        // Shoulder SparkMax settings
        SparkMaxConfig shoulderConfig = new SparkMaxConfig();
        shoulderConfig.idleMode(IdleMode.kBrake);
        shoulderConfig.inverted(false);
        shoulderConfig.absoluteEncoder.zeroOffset(ArmConstants.SHOULDER_ABSOLUTE_ENCODER_ZERO_OFFSET);
        shoulderConfig.absoluteEncoder.inverted(false);
        shoulderMotor.configure(shoulderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Wrist SparkMax settings
        SparkMaxConfig wristConfig = new SparkMaxConfig();
        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Simple arm angle controller
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


    // Simple wrist angle controller
    public void moveWristToSetpoint(double setpoint) {
        wristSetpoint = setpoint;
        double error     = setpoint - getWristEncoderPosition();
        double PIDoutput = error * ArmConstants.kWristP;
        PIDoutput = Math.min(Math.abs(PIDoutput), ArmConstants.MAX_WRIST_SPEED) * Math.signum(PIDoutput);
        setWristSpeed(PIDoutput);
    }

    /*
     * WARNING:
     * Coral sensor is blocked by coral stabilizer bracket in the intake
     * Will not provide good results
     * Do not use while bracket is still in place
     */
    public boolean hasGamePiece() {
        return !intakeSensor.get();
    }

    // Will stop manual control from hitting the chassis with the arm
    public void isGroundLock(Boolean groundLock) {
        isGroundLock = groundLock;
    }

    // Set the direction and speed of the intake
    public void setIntakeSpeed(double intakeSpeed, boolean isReversed) {
        intakeMotor.set(VictorSPXControlMode.PercentOutput, isReversed ? -intakeSpeed : intakeSpeed);
        System.out.println("Intaking...");
    }

    public void stopWrist() {
        wristMotor.stopMotor();
    }

    public void setWristSpeed(double speed) {
        wristMotor.set(speed);
    }

    // Halt all control to the arm
    public void stop() {
        shoulderMotor.stopMotor();
        stopWrist();
    }

    public double getShoulderEncoderPosition() {
        return shoulderEncoder.getPosition();
    }

    public double getShoulderAngle() {
        return ((shoulderAbsoluteEncoder.getPosition() / ArmConstants.SHOULDER_GEAR_RATIO) * 360) - ArmConstants.SHOULDER_OFFSET;
    }

    public double getWristEncoderPosition() {
        return wristEncoder.getPosition();
    }

    @Override
    public void periodic() {

        // Update SmartDashboard
        SmartDashboard.putNumber("Arm/Shoulder Angle", getShoulderAngle());
        SmartDashboard.putBoolean("Intake/Game Piece Detected", hasGamePiece());
        SmartDashboard.putBoolean("Arm/GroundLocked", isGroundLock);
        SmartDashboard.putNumber("Arm/Shoulder Setpoint", shoulderSetpoint);
        SmartDashboard.putNumber("Arm/Wrist Position", getWristEncoderPosition());
        SmartDashboard.putNumber("Arm/Wrist Setpoint", wristSetpoint);
    }
}
