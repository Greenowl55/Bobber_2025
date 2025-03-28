package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class Tilt extends SubsystemBase {

    private final TalonFX m_tilt = new TalonFX(Constants.TILT_MOTOR_ID);
    private double angle;
    private double speed;
    
    
        public Tilt() {
            SmartDashboard.putData(this);
            var talonFXconfigs = new TalonFXConfiguration();
    
            // in init function, set slot 0 gains
            var slot0Configs = talonFXconfigs.Slot0;
            slot0Configs.kS = 0.87; // voltage needed to overcome static friction
            slot0Configs.kV = 0.03; // output per unit of target velocity (output/rps)
            slot0Configs.kA = 0.1; // output per unit of target acceleration (output/rps^2)
            slot0Configs.kP = 10; // output per unit of error in position (output/rotation), An error of 1
                                    // rotation results in 2.4 V output
            slot0Configs.kI = 0; // output per unit of integrated error in position (output/(rotation*s)), no
                                 // output form integrated error
            slot0Configs.kD = 0; // output per unit of error in velocity (output/rps), A velocity of 1 rps
                                     // results in 0.1 V output
    
            var motionMagicConfigs = talonFXconfigs.MotionMagic;
            motionMagicConfigs.MotionMagicCruiseVelocity = 300; // velocity in units/100ms
            motionMagicConfigs.MotionMagicAcceleration = 150; // acceleration in units/100ms^2 (2x the speed of the cruise velocity)
            motionMagicConfigs.MotionMagicJerk = 500; // jerk in units/100ms^3 (10x the speed of the cruise velocity)
    
            m_tilt.getConfigurator().apply(talonFXconfigs);
            m_tilt.setNeutralMode(NeutralModeValue.Brake);
        }
    
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

        public void setAngle(double angle) {
            this.angle = angle;
            m_tilt.setControl(m_request.withPosition(angle));
        }
    
        public void setspeed(double speed){
            this.speed = speed;
            m_tilt.set(speed);
    }

    public double getPosition() {
        return m_tilt.getPosition().getValueAsDouble();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("TiltConnected", () -> m_tilt.isConnected(), null);
        builder.addBooleanProperty("TiltAlive", () -> m_tilt.isAlive(), null);
        builder.addDoubleProperty("TiltAngle", () -> this.angle, null);
    }
}