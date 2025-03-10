package frc.robot.subsystems;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
public class Climber extends SubsystemBase{


    private final TalonFX m_Climb = new TalonFX(Constants.CLIMBER_MOTOR_ID);
    private double angle;

    public Climber() {
        var talonFXconfigs = new TalonFXConfiguration();

        // in init function, set slot 0 gains
        var slot0Configs = talonFXconfigs.Slot0;
        slot0Configs.kS = 0.2; // voltage needed to overcome static friction
        slot0Configs.kV = 0.1; // output per unit of target velocity (output/rps)
        slot0Configs.kA = 0.1; // output per unit of target acceleration (output/rps^2)
        slot0Configs.kP = 0.01; // output per unit of error in position (output/rotation), An error of 1
                                // rotation results in 2.4 V output
        slot0Configs.kI = 0; // output per unit of integrated error in position (output/(rotation*s)), no
                             // output form integrated error
        slot0Configs.kD = 0.001; // output per unit of error in velocity (output/rps), A velocity of 1 rps
                                 // results in 0.1 V output

        var motionMagicConfigs = talonFXconfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 1000; // velocity in units/100ms
        motionMagicConfigs.MotionMagicAcceleration = 1000; // acceleration in units/100ms^2
        motionMagicConfigs.MotionMagicJerk = 1000; // jerk in units/100ms^3

        m_Climb.getConfigurator().apply(talonFXconfigs);
    }

    public void setAngle(double angle) {
        this.angle = angle;
        m_Climb.setControl(new PositionVoltage(angle).withSlot(0));
    }

    public void setSpeed(double speed) {
        m_Climb.set(speed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("TiltConnected", () -> m_Climb.isConnected(), null);
        builder.addBooleanProperty("TiltAlive", () -> m_Climb.isAlive(), null);
        builder.addDoubleProperty("TiltAngle", () -> this.angle, null);
    }
}