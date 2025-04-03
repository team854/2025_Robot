package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class DefaultArmCommand extends Command {

    private final ArmSubsystem   armSubsystem;
    private final RobotContainer robotContainer;
    private double               shoulderSetpoint;
    private boolean              useLowerLimit17 = true; // Toggle state variable

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
        armSubsystem.setWristSpeed(wristSpeed * ArmConstants.MAX_WRIST_SPEED);

        double lowerLimit = useLowerLimit17 ? 17 : 52;
        shoulderSetpoint = Math.min(155, Math.max(lowerLimit,
            shoulderSetpoint + ArmConstants.MAX_DEGREES_PER_LOOP * robotContainer.getShoulderSpeed()));

        armSubsystem.moveShoulderToSetpoint(shoulderSetpoint);
    }

    public void setShoulderSetpoint(double newSetpoint) {
        this.shoulderSetpoint = newSetpoint;
    }

    public void toggleLowerLimit() {
        useLowerLimit17 = !useLowerLimit17;
        System.out.println("----------TUCKED MODE TOGGLED----------");
        System.out.println("Arm lower limit set to: " + (useLowerLimit17 ? 17 : 52));
    }

    @Override
    public boolean isFinished() {
        return false; // Default commands never end
    }
}
