package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorPosition;
import frc.robot.commands.tilt.Tilt_Position;
import frc.robot.commands.vision.*;

public class mid1pc extends SequentialCommandGroup{
    public mid1pc (CommandSwerveDrivetrain swerve, Elevator elevator, Tilt tilt, Coral coral){
        addCommands(
            new ElevatorPosition(elevator, "intake", Constants.ELEVATOR_INTAKE),
            new ReefLeft(swerve, null),
            new ElevatorPosition(elevator, getName(), Constants.ELEVATOR_L4),
            wait(0),
            new Tilt_Position(tilt, getName(), 0)

        );
    }
}
