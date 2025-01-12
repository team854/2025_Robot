package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    /**
     * NavX - AHRS
     *
     * This local NavXGryro is used to override the value in the gyro dashboard sendable to use
     * {@link AHRS#getAngle()} which includes any offset, instead of the {@link AHRS#getYaw()} which
     * is the raw yaw value without the offset.
     * <p>
     * Using the getAngle() method makes the gyro appear in the correct position on the dashboard
     * accounting for the offset.
     */
    private class NavXGyro extends AHRS {
        private NavXGyro() {
            super(NavXComType.kMXP_SPI);
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value",
                () -> {
                    double angle = super.getAngle();
                    // Print the angle in the range 0-360;
                    angle %= 360;
                    if (angle < 0) {
                        angle += 360;
                    }
                    // Round the angle to 2 decimal places for the Dashboard
                    return Math.round(angle * 100d) / 100d;
                },
                null);
        }
    }

    private final LightsSubsystem     lightsSubsystem;

    // The motors on the left side of the drive.
    private final SparkMax            leftPrimaryMotor   = new SparkMax(DriveConstants.LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkMax            leftFollowerMotor  = new SparkMax(DriveConstants.LEFT_MOTOR_CAN_ID + 1,
        MotorType.kBrushless);

    // The motors on the right side of the drive.
    private final SparkMax            rightPrimaryMotor  = new SparkMax(DriveConstants.RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkMax            rightFollowerMotor = new SparkMax(DriveConstants.RIGHT_MOTOR_CAN_ID + 1,
        MotorType.kBrushless);

    private double                    leftSpeed          = 0;
    private double                    rightSpeed         = 0;

    // Encoders
    private final RelativeEncoder     leftEncoder        = leftPrimaryMotor.getEncoder();
    private final RelativeEncoder     rightEncoder       = rightPrimaryMotor.getEncoder();

    private double                    leftEncoderOffset  = 0;
    private double                    rightEncoderOffset = 0;

    /*
     * Gyro
     */
    private NavXGyro                  navXGyro           = new NavXGyro();

    private double                    gyroHeadingOffset  = 0;
    private double                    gyroPitchOffset    = 0;

    /*
     * Simulation fields
     */
    private Field2d                   field              = null;
    private DifferentialDrivetrainSim drivetrainSim      = null;
    private double                    simAngle           = 0;
    private double                    simLeftEncoder     = 0;
    private double                    simRightEncoder    = 0;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem(LightsSubsystem lightsSubsystem) {

        this.lightsSubsystem = lightsSubsystem;

        /*
         * Configure Left Side Motors
         */
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(DriveConstants.LEFT_MOTOR_INVERTED)
            .idleMode(IdleMode.kBrake)
            .disableFollowerMode();
        leftPrimaryMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.follow(leftPrimaryMotor);
        leftFollowerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        /*
         * Configure Right Side Motors
         */
        config = new SparkMaxConfig();
        config.inverted(DriveConstants.RIGHT_MOTOR_INVERTED)
            .idleMode(IdleMode.kBrake)
            .disableFollowerMode();
        rightPrimaryMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.follow(rightPrimaryMotor);
        rightFollowerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetEncoders();

        // Reset the Gyro Heading
        resetGyro();

        // Add the field elements for robot simulations
        if (RobotBase.isSimulation()) {

            field = new Field2d();
            SmartDashboard.putData("Field", field);

            drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
                KitbotMotor.kDoubleNEOPerSide, // Double NEO per side
                KitbotGearing.k10p71, // 10.71:1
                KitbotWheelSize.kSixInch, // 6" diameter wheels.
                null // No measurement noise.
            );
        }
    }

    /**
     * Reset Gyro
     * <p>
     * This routine resets the gyro angle to zero.
     * <p>
     * NOTE: This is not the same as calibrating the gyro.
     */
    public void resetGyro() {

        setGyroHeading(0);
        setGyroPitch(0);
    }

    /**
     * Set Gyro Heading
     * <p>
     * This routine sets the gyro heading to a known value.
     */
    public void setGyroHeading(double heading) {

        // Clear the current offset.
        gyroHeadingOffset = 0;

        // Adjust the offset so that the heading is now the current heading.
        gyroHeadingOffset = heading - getHeading();

        // Send the offset to the navX in order to have the
        // compass on the dashboard appear at the correct heading.
        navXGyro.setAngleAdjustment(gyroHeadingOffset);
    }

    /**
     * Set Gyro Pitch
     * <p>
     * This routine sets the gyro pitch to a known value.
     */
    public void setGyroPitch(double pitch) {

        // Clear the current offset.
        gyroPitchOffset = 0;

        // Adjust the offset so that the heading is now the current heading.
        gyroPitchOffset = pitch - getPitch();
    }

    /**
     * Gets the heading of the robot.
     *
     * @return heading in the range of 0 - 360 degrees
     */
    public double getHeading() {

        double gyroYawAngle = navXGyro.getYaw();

        // Add the simulated angle to support simulation
        gyroYawAngle += simAngle;

        if (DriveConstants.GYRO_INVERTED) {
            gyroYawAngle *= -1;
        }

        // adjust by the offset that was saved when the gyro
        // heading was last set.
        gyroYawAngle += gyroHeadingOffset;

        // Round to two decimal places
        gyroYawAngle  = Math.round(gyroYawAngle * 100) / 100;

        // The angle can be positive or negative and extends beyond 360 degrees.
        double heading = gyroYawAngle % 360.0;

        if (heading < 0) {
            heading += 360;
        }

        // round to two decimals
        return heading;
    }

    /**
     * Get the error between the current heading and the requested heading in the
     * range -180 to +180 degrees.
     * <p>
     * A positive result means that the passed in heading is clockwise from the
     * current heading.
     *
     * @param requiredHeading to measure the heading error
     * @return degrees difference between the required heading and the current heading.
     */
    public double getHeadingError(double requiredHeading) {

        double currentHeading = getHeading();

        // Determine the error between the current heading and
        // the desired heading
        double error          = requiredHeading - currentHeading;

        if (error > 180) {
            error -= 360;
        }
        else if (error < -180) {
            error += 360;
        }

        return error;
    }

    public double getPitch() {

        double gyroPitch = navXGyro.getPitch();

        // adjust by the offset that was saved when the gyro
        // pitch was last set.
        gyroPitch += gyroPitchOffset;

        // round to two decimals
        return Math.round(gyroPitch * 100) / 100d;
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderValue() {
        return (getLeftEncoder() + getRightEncoder()) / 2;
    }

    public double getEncoderDistanceCm() {

        return getAverageEncoderValue() * DriveConstants.CM_PER_ENCODER_COUNT;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public double getLeftEncoder() {
        return leftEncoder.getPosition() + simLeftEncoder + leftEncoderOffset;
    }

    /**
     * Gets the left velocity.
     *
     * @return the left drive encoder speed
     */
    public double getLeftEncoderSpeed() {
        return leftEncoder.getVelocity();
    }

    /**
     * Gets the right velocity.
     *
     * @return the right drive encoder speed
     */
    public double getRightEncoderSpeed() {
        return rightEncoder.getVelocity();
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public double getRightEncoder() {
        return rightEncoder.getPosition() + simRightEncoder + rightEncoderOffset;
    }

    /** Resets the drive encoders to zero. */
    public void resetEncoders() {

        // Reset the offsets so that the encoders are zeroed.
        leftEncoderOffset  = 0;
        leftEncoderOffset  = -getLeftEncoder();

        rightEncoderOffset = 0;
        rightEncoderOffset = -getRightEncoder();
    }

    /**
     * Set the left and right speed of the primary and follower motors
     *
     * @param leftSpeed
     * @param rightSpeed
     */
    public void setMotorSpeeds(double leftSpeed, double rightSpeed) {

        this.leftSpeed  = leftSpeed;
        this.rightSpeed = rightSpeed;

        // NOTE: Follower motors are set to follow the primary motors
        leftPrimaryMotor.set(this.leftSpeed);
        rightPrimaryMotor.set(this.rightSpeed);
    }

    /** Safely stop the subsystem from moving */
    public void stop() {
        setMotorSpeeds(0, 0);
    }

    @Override
    public void periodic() {

        lightsSubsystem.setDriveMotorSpeeds(leftSpeed, rightSpeed);

        SmartDashboard.putNumber("Right Motor", rightSpeed);
        SmartDashboard.putNumber("Left  Motor", leftSpeed);

        SmartDashboard.putNumber("Right Encoder", Math.round(getRightEncoder() * 100) / 100d);
        SmartDashboard.putNumber("Left Encoder", Math.round(getLeftEncoder() * 100) / 100d);
        SmartDashboard.putNumber("Avg Encoder", Math.round(getAverageEncoderValue() * 100) / 100d);
        SmartDashboard.putNumber("Distance (cm)", Math.round(getEncoderDistanceCm() * 10) / 10d);
        SmartDashboard.putNumber("Right Velocity", Math.round(getRightEncoderSpeed() * 100) / 100d);
        SmartDashboard.putNumber("Left Velocity", Math.round(getLeftEncoderSpeed() * 100) / 100d);

        SmartDashboard.putData("Gyro", navXGyro);
        SmartDashboard.putNumber("Gyro Heading", getHeading());
        SmartDashboard.putNumber("Gyro Pitch", getPitch());

    }

    @Override
    public void simulationPeriodic() {

        if (RobotController.isSysActive()) {

            // When the robot is enabled, calculate the position
            // Set the inputs to the system.
            drivetrainSim.setInputs(
                leftSpeed * RobotController.getInputVoltage(),
                rightSpeed * RobotController.getInputVoltage());

            // Advance the model by 20 ms. Note that if you are running this
            // subsystem in a separate thread or have changed the nominal timestep
            // of TimedRobot, this value needs to match it.
            drivetrainSim.update(0.02);

            // Move the robot on the simulated field
            field.setRobotPose(drivetrainSim.getPose());
        }
        else {
            // When the robot is disabled, allow the user to move
            // the robot on the simulation field.
            drivetrainSim.setPose(field.getRobotPose());
        }

        // Update the gyro simulation offset
        // NOTE: the pose has the opposite rotational direction from the system
        // pose degrees are counter-clockwise positive. weird.
        simAngle        = -drivetrainSim.getPose().getRotation().getDegrees();

        // Update the encoders with the simulation offsets.
        simLeftEncoder  = drivetrainSim.getLeftPositionMeters() * 100 / DriveConstants.CM_PER_ENCODER_COUNT;
        simRightEncoder = drivetrainSim.getRightPositionMeters() * 100 / DriveConstants.CM_PER_ENCODER_COUNT;
    }

    @Override
    public String toString() {

        StringBuilder sb = new StringBuilder();

        sb.append(this.getClass().getSimpleName()).append(" : ")
            .append("Heading ").append(getHeading())
            .append(", Pitch ").append(getPitch())
            .append(", Drive dist ").append(Math.round(getEncoderDistanceCm() * 10) / 10d).append("cm");

        return sb.toString();
    }
}
