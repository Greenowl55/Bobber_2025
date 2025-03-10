package frc.robot.commands.Climb;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberPosition extends Command {
    private Climber m_climber;
    private double position;

    public ClimberPosition(Climber climb, String pos, double position) {
        addRequirements(climb);
        this.m_climber = climb;
        this.position = position;
        SmartDashboard.putData("TiltPosition_" + pos, this);
    }

    @Override
    public void initialize() {
        this.m_climber.setAngle(this.position);
    }
    
    @Override
    public void execute() {
    }

  public boolean isFinished() {
        if (m_climber.getPosition() == this.position){
            return true;
        } else{
            return false;
        }
    }
    
    @Override
    public void end(boolean isFinished) {
        m_climber.setAngle(Constants.CLIMBER_IN);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Position", () -> this.position, (val) -> this.position = val);
    }

  

}