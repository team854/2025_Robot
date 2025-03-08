package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class PrecisionSwerveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private double                preciseSpeed;   // Converted to encoder units
    private double                preciseRotation;

    public PrecisionSwerveCommand(SwerveSubsystem swerveSubsystem, double preciseSpeed,
        double preciseRotation) {
        this.swerveSubsystem = swerveSubsystem;
        this.preciseSpeed    = preciseSpeed;
        this.preciseRotation = preciseRotation;

        // Declare subsystem dependencies
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        swerveSubsystem.slowSpeed(preciseSpeed, preciseRotation);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.slowSpeed(OperatorConstants.MAX_SPEED, OperatorConstants.SWERVE_ROTATION_SCALE);

    }
}
