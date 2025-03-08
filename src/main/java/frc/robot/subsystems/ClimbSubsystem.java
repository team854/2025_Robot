package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    public final VictorSPX climbMotor;

    public ClimbSubsystem() {
        climbMotor = new VictorSPX(ClimbConstants.CLIMB_MOTOR_ID);
    }

    public void setClimbSpeed(double setSpeed) {
        climbMotor.set(VictorSPXControlMode.PercentOutput, setSpeed);
    }
}
