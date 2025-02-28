package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeCommand extends Command {
    private final ArmSubsystem armSubsystem;
    public boolean             isReversed;
    public double              intakeSpeed;

    public IntakeCommand(ArmSubsystem armSubsystem, Boolean isReversed, Double intakeSpeed) {
        this.armSubsystem = armSubsystem;
        this.intakeSpeed  = intakeSpeed;
        this.isReversed   = isReversed;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        isReversed = false;
    }

    @Override
    public void execute() {

        // Reversed direction rotates wheels inwards
        isReversed = true;
        armSubsystem.setIntakeSpeed(intakeSpeed, isReversed);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setIntakeSpeed(intakeSpeed, isReversed);
    }
}
