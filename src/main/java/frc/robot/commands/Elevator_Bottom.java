package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;


public class Elevator_Bottom extends Command {

    private final Elevator m_Elevator;

    public Elevator_Bottom(Elevator Subsystems) {
        m_Elevator = Subsystems;
        addRequirements(m_Elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_Elevator.setPosition(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
