package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ZeroElevator extends Command {
    private Elevator elevator;
    public ZeroElevator(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }
    @Override
    public void end(boolean interrupted) {
        elevator.setBrakeMode(true);
        elevator.zeroElevatorPosition();
    }
    @Override
    public void initialize() {
        elevator.setBrakeMode(false);
    }


    
}
