package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;

public class Rollout extends Command {
    private double speed;
    private Algae algae;

    public Rollout(Algae algae, double speed) {
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
