package frc.robot.commands.tilt;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Tilt;

public class AutoTilt extends Command {
    private Tilt m_tilt;
    private double position;

    public AutoTilt(Tilt tilt, double position) {
        addRequirements(tilt);
        this.m_tilt = tilt;
        this.position = position;
    }

    @Override
    public void initialize() {
        this.m_tilt.setAngle(this.position);
    }

    @Override
    public void execute() {
    }

    public boolean isFinished() {
        double distance = Math.abs(m_tilt.getPosition() - this.position);
        if (distance <= Constants.MOTOR_POSITION_EPSILON) {
            return true;
        } else {
            return false;
        }
    }

}
