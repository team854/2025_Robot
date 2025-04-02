package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class DefaultArmCommand extends Command {

    private final ArmSubsystem   armSubsystem;
    private final RobotContainer robotContainer;
    private double               shoulderSetpoint;

    public DefaultArmCommand(RobotContainer robotContainer, ArmSubsystem armSubsystem) {

        this.robotContainer = robotContainer;
        this.armSubsystem   = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        shoulderSetpoint = armSubsystem.getShoulderAngle();
    }

    @Override
    public void execute() {

        double wristSpeed = robotContainer.getWristSpeed();

        /*
         * Set the wrist speed
         */
        armSubsystem.setWristSpeed(wristSpeed * ArmConstants.MAX_WRIST_SPEED);

        /*
         * Set the shoulder speed
         * 5
         * The shoulder speed has different max up speed and down speed.
         */

        shoulderSetpoint = Math.min(155,
            Math.max(10, shoulderSetpoint + ArmConstants.MAX_DEGREES_PER_LOOP * robotContainer.getShoulderSpeed()));


        armSubsystem.setShoulderSetpoint(shoulderSetpoint);
    }

    @Override
    public boolean isFinished() {
        // Default Commands never end
        return false;
    }
}
