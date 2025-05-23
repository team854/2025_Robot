package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmAngleCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final double       targetAngle;

    public SetArmAngleCommand(ArmSubsystem armSubsystem, double targetAngle) {
        this.armSubsystem = armSubsystem;
        this.targetAngle  = targetAngle;
    }

    @Override
    public void initialize() {
        System.out.println("Setting arm angle to " + targetAngle);
    }

    @Override
    public void execute() {
        armSubsystem.moveShoulderToSetpoint(targetAngle);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stop();

        // Update DefaultArmCommand with the last target angle
        if (armSubsystem.getDefaultCommand() instanceof DefaultArmCommand) {
            ((DefaultArmCommand) armSubsystem.getDefaultCommand()).setShoulderSetpoint(targetAngle);
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(targetAngle - armSubsystem.getShoulderAngle()) < 20;
    }
}
