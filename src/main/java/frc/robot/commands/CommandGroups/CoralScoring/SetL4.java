package frc.robot.commands.CommandGroups.CoralScoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.Arm.SetArmAngleCommand;
import frc.robot.commands.Arm.SetWristPositionCommand;
import frc.robot.commands.Elevator.SetElevatorHeightCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetL4 extends ParallelCommandGroup {

    public SetL4(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {

        addCommands(

            /*
             * Set elevator to L4 setpoint
             */
            new SetElevatorHeightCommand(elevatorSubsystem, ElevatorConstants.UPPER_ELEVATOR_L4_SETPOINT,
                ElevatorConstants.LOWER_ELEVATOR_L4_SETPOINT),

            /*
             * Set wrist to vertical position
             */
            new SetWristPositionCommand(armSubsystem, ArmConstants.WRIST_VERTICAL_DEGREES),

            /*
             * Set arm to L4 angle
             */
            new SetArmAngleCommand(armSubsystem, ArmConstants.ARM_L4_ANGLE));
    }


}
