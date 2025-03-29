package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ManualControl extends Command {
    private Elevator elevator;
    private double speed;


    public ManualControl(Elevator elevator, double speed) {
        this.elevator = elevator;
        this.speed = speed;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        this.elevator.elevator(this.speed);
    }

    @Override
    public void end(boolean interrupted) {
        this.elevator.elevator(0);
    }

}