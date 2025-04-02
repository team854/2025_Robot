package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmAngleCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final double       targetAngle;

    public SetArmAngleCommand(ArmSubsystem armSubsystem, double targetAngle) {
        this.armSubsystem = armSubsystem;
        this.targetAngle  = targetAngle;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Setting arm angle to " + targetAngle + " degrees");
        // Set the target setpoint in the subsystem's PID controller
        armSubsystem.setShoulderSetpoint(targetAngle);
    }

    @Override
    public void execute() {
        // Continuously update the setpoint
        armSubsystem.setShoulderSetpoint(targetAngle);
    }

    @Override
    public boolean isFinished() {
        // The command finishes once the shoulder is at the setpoint (within tolerance)
        return armSubsystem.isShoulderAtSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("SetArmAngleCommand interrupted");
        }
        else {
            System.out.println("Arm reached target angle: " + targetAngle + " degrees");
        }
        // Optionally, stop the shoulder motor when finished.
        // armSubsystem.stop();
    }
}
