package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Fish_Hook;


public class Elevator_Bottom extends SequentialCommandGroup {

    private final Elevator Elevator;
    private final Fish_Hook Fish_Hook;

    public Elevator_Bottom(Elevator m_Elevator, Fish_Hook m_Fish_Hook) { 
        Elevator = m_Elevator;
        Fish_Hook = m_Fish_Hook;
        addRequirements(m_Elevator, m_Fish_Hook);

        addCommands(
            Elevator.runOnce(() -> Elevator.setPosition(0)),
            Fish_Hook.runOnce(() -> Fish_Hook.setposition(0))
            //in theory this will put the elevator and fishook in starting config
        );
    }    
}
