// package frc.robot.commands.Arm;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.Tolerances;
// import frc.robot.subsystems.ArmSubsystem;

// public class SetArmAngleCommand extends Command {
// private final ArmSubsystem armSubsystem;
// private final double targetAngle;

// public SetArmAngleCommand(ArmSubsystem armSubsystem, double targetAngle) {
// this.armSubsystem = armSubsystem;
// this.targetAngle = targetAngle;

// // Declare subsystem dependencies
// addRequirements(armSubsystem);
// }

// @Override
// public void initialize() {
// armSubsystem.moveShoulderToSetpoint(targetAngle);
// }

// @Override
// public void execute() {
// armSubsystem.moveShoulderToSetpoint(targetAngle);
// }

// @Override
// public boolean isFinished() {
// return armSubsystem.getShoulderEncoderPosition() >= targetAngle - Tolerances.SHOULDER_LOWER_TOLERANCE &&
// armSubsystem.getShoulderEncoderPosition() < targetAngle + Tolerances.SHOULDER_UPPER_TOLERANCE;
// }
// }
