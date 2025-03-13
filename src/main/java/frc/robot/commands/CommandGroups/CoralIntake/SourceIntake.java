package frc.robot.commands.CommandGroups.CoralIntake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.Arm.IntakeCommand;
import frc.robot.commands.Arm.SetArmAngleCommand;
import frc.robot.commands.Arm.SetWristPositionCommand;
import frc.robot.commands.Elevator.SetElevatorHeightCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SourceIntake extends SequentialCommandGroup {

    public SourceIntake(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {

        addCommands(

            /*
             * Begin intaking
             */
            new IntakeCommand(armSubsystem, true, ArmConstants.INTAKE_SOURCE_SPEED),

            /*
             * Set arm to source angle
             */
            new SetArmAngleCommand(armSubsystem, ArmConstants.ARM_GROUND_ANGLE),

            /*
             * Rotate wrist to horizontal
             */
            new SetWristPositionCommand(armSubsystem, ArmConstants.WRIST_HORIZONTAL_DEGREES),

            /*
             * Set elevator to default setpoint
             */
            new SetElevatorHeightCommand(elevatorSubsystem, ElevatorConstants.UPPER_ELEVATOR_SOURCE_SETPOINT,
                ElevatorConstants.LOWER_ELEVATOR_SOURCE_SETPOINT));



    }


}
