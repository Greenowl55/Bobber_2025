package frc.robot.subsystems;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
		slot0Configs.kS = 0.56; // voltage needed to overcome static friction
		slot0Configs.kV = 0.1219; // output per unit of target velocity (output/rps)
		slot0Configs.kA = 0.02; // output per unit of target acceleration (output/rps^2)
		slot0Configs.kP = 3; // output per unit of error in position (output/rotation)
		slot0Configs.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
		slot0Configs.kD = 0; // output per unit of error in velocity (output/rps)

		var motionMagicConfigs = talonFXconfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = 50; // velocity in units/100ms
		motionMagicConfigs.MotionMagicAcceleration = 100; // acceleration in units/100ms^2
		motionMagicConfigs.MotionMagicJerk = 500; // jerk in units/100ms^3

        m_Climb.getConfigurator().apply(talonFXconfigs);
    }

    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    public void setAngle(double angle) {
        this.angle = angle;
        m_Climb.setControl(m_request.withPosition(angle));
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

    public double getPosition() {
        return m_Climb.getPosition().getValueAsDouble();
    }
}