package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetWristPositionCommand extends Command {
    public final ArmSubsystem armSubsystem;
    public final double       wristSetpoint;

    public SetWristPositionCommand(ArmSubsystem armSubsystem, double wristSetpoint) {
        this.armSubsystem  = armSubsystem;
        this.wristSetpoint = wristSetpoint;

        // addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.moveWristToSetpoint(wristSetpoint);
        System.out.println("Moving wrist to: " + wristSetpoint);

    }

    @Override
    public void execute() {
    }

}
