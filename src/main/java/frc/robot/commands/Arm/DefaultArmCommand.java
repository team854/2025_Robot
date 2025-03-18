package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class DefaultArmCommand extends Command {

    private final ArmSubsystem   armSubsystem;
    private final RobotContainer robotContainer;

    public DefaultArmCommand(RobotContainer robotContainer, ArmSubsystem armSubsystem) {

        this.robotContainer = robotContainer;
        this.armSubsystem   = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {

        double wristSpeed    = robotContainer.getWristSpeed();
        double shoulderSpeed = robotContainer.getShoulderSpeed();

        /*
         * Set the wrist speed
         */
        armSubsystem.setWristSpeed(wristSpeed * ArmConstants.MAX_WRIST_SPEED);

        /*
         * Set the shoulder speed
         *
         * The shoulder speed has different max up speed and down speed.
         */

        double shoulderMotorSpeed = 0;

        if (shoulderSpeed > 0) {
            // Always add the hold speed when going up
            shoulderMotorSpeed += armSubsystem.getShoulderHoldSpeed();
            shoulderMotorSpeed += ArmConstants.MAX_SHOULDER_UP_SPEED * shoulderSpeed;
        }
        else {
            // Only apply a down or hold current if the arm angle is greater than 45 deg above the resting position.
            // Below that, let the arm drift down to save the motor from overheating.
            shoulderMotorSpeed += armSubsystem.getShoulderHoldSpeed();
            shoulderMotorSpeed += Math.abs(ArmConstants.MAX_SHOULDER_DOWN_SPEED) * shoulderSpeed;
        }

        armSubsystem.setShoulderSpeed(shoulderMotorSpeed);
    }

    @Override
    public boolean isFinished() {
        // Default Commands never end
        return false;
    }
}
