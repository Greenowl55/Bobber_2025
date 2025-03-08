package frc.robot.ng.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ng.subsystems.FishHook;

public class CoralManual extends Command {

    private final FishHook fishHook;

    public CoralManual(FishHook fishhook) {
        fishHook = fishhook;
        addRequirements(fishHook);
    }

    @Override
    public void end(boolean interrupted) {
        fishHook.intakeIdle();
    }

    @Override
    public void initialize() {
        fishHook.intakeManual();
    }
}
