package frc.robot.commands.tilt;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Tilt;

public class Tilt_Position extends Command {
    private Tilt m_tilt;
    private double position;

    public Tilt_Position(Tilt tilt, String pos, double position) {
        addRequirements(tilt);
        this.m_tilt = tilt;
        this.position = position;
        SmartDashboard.putData("TiltPosition_" + pos, this);
    }

    @Override
    public void initialize() {
        this.m_tilt.setAngle(this.position);
    }
    
    @Override
    public void execute() {
    }

  public boolean isFinished() {
        if (m_tilt.getPosition() == this.position){
            return true;
        } else{
            return false;
        }
    }
    
    @Override
    public void end(boolean isFinished) {
        m_tilt.setAngle(Constants.FISHHOOK_IN);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Position", () -> this.position, (val) -> this.position = val);
    }

  

}