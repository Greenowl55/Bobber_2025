package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorPosition extends Command {
    private Elevator elevator;
    private double position;

    public ElevatorPosition(Elevator elevator, double position) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.position = position;
    }

    @Override
    public void initialize() {
        this.elevator.setPosition(this.position);
    }

    @Override
    public boolean isFinished() {
        double distance = Math.abs(this.elevator.getPosition() - this.position);
        if (distance <= Constants.MOTOR_POSITION_EPSILON) {
            return true;
        } else {
            return false;
        }
    }
}
