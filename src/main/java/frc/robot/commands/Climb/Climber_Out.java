package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Climber_Out extends Command{
    private double speed;
    private Climber climber;

    public Climber_Out(Climber climber, double speed) {
        this.climber = climber;
        this.speed = speed;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        this.climber.setSpeed(this.speed);
    }

    @Override
    public void end(boolean interrupted) {
        this.climber.setSpeed(0);
    }
}
