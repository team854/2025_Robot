package frc.robot.commands.CommandGroups.CoralScoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.Arm.SetArmAngleCommand;
import frc.robot.commands.Arm.SetWristPositionCommand;
import frc.robot.commands.Elevator.SetElevatorHeightCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetL3 extends ParallelCommandGroup {

    public SetL3(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {

        addCommands(

            /*
             * Set elevator to L3 setpoint
             */
            new SetElevatorHeightCommand(elevatorSubsystem, ElevatorConstants.UPPER_ELEVATOR_L3_SETPOINT,
                ElevatorConstants.LOWER_ELEVATOR_L3_SETPOINT),

            /*
             * Set wrist to vertical position
             */
            new SetWristPositionCommand(armSubsystem, ArmConstants.WRIST_VERTICAL_ANGLE),

            /*
             * Set arm to L3 angle
             */
            new SetArmAngleCommand(armSubsystem, ArmConstants.ARM_L3_ANGLE));
    }


}
