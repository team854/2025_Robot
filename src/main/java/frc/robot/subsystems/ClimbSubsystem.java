package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    public final VictorSPX climbMotor;

    public ClimbSubsystem() {
        climbMotor = new VictorSPX(ClimbConstants.CLIMB_MOTOR_ID);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Motor Velocity: ", getClimbSpeed());
    }

    public double getClimbSpeed() {
        return climbMotor.getActiveTrajectoryVelocity();
    }

    public void setClimbPosition(double setSpeed) {
        climbMotor.set(VictorSPXControlMode.Velocity, setSpeed);
    }
}
