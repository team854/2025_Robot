package frc.robot.commands.CommandGroups.CoralScoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.Arm.SetArmAngleCommand;
import frc.robot.commands.Arm.SetWristPositionCommand;
import frc.robot.commands.Elevator.SetElevatorHeightCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetL2 extends ParallelCommandGroup {

    public SetL2(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem,
        SetElevatorHeightCommand setElevatorHeightCommand,
        SetArmAngleCommand setArmAngleCommand,
        SetWristPositionCommand setWristPositionCommand) {

        addCommands(

            /*
             * Set elevator to L2 setpoint
             */
            new SetElevatorHeightCommand(elevatorSubsystem, ElevatorConstants.UPPER_ELEVATOR_L2_SETPOINT,
                ElevatorConstants.LOWER_ELEVATOR_L2_SETPOINT),

            /*
             * Set wrist to vertical position
             */
            new SetWristPositionCommand(armSubsystem, ArmConstants.WRIST_VERTICAL_DEGREES),

            /*
             * Set arm to L2 angle
             */
            new SetArmAngleCommand(armSubsystem, ArmConstants.ARM_L2_ANGLE));
    }


}
