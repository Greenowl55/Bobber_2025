package frc.robot.ng.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ng.subsystems.FishHook;

public class AlgaeIn extends Command {

    private final FishHook fishHook;

    public AlgaeIn(FishHook fishHook) {
        this.fishHook = fishHook;
        this.addRequirements(fishHook);
    }

    @Override
    public void end(boolean interrupted) {
        fishHook.algaeIdle();
    }

    @Override
    public void initialize() {
        fishHook.algaeRollIn();
    }

}
