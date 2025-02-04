package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;


public class Elevator_L1 extends SequentialCommandGroup {

    private final Elevator Elevator;

    public Elevator_L1(Elevator m_Elevator) {
        Elevator = m_Elevator;
        addRequirements(m_Elevator);

        addCommands(
            Elevator.runOnce(() -> Elevator.setPosition(100/*Change this to the correct value later*/))
            //Move elevator to position 1
            //Angle down Fish hook
            //Run coral motors
        );
    }
}
