package frc.robot.NewCommands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.NewSubsytems.Algae;

public class RollIn extends Command {
    private double speed;
    private Algae algae;

    public RollIn(Algae algae, double speed) {
        this.speed = speed;
        this.algae = algae;
        addRequirements(algae);
    }

    @Override
    public void initialize() {
        this.algae.setSpeed(this.speed);
    }

    @Override
    public void end(boolean interrupted) {
        this.algae.setSpeed(0);
    }
}