package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.security.PrivateKey;

import com.ctre.phoenix6.signals.PIDRefSlopeECUTime_ClosedLoopModeValue;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Leds extends SubsystemBase{

private AddressableLED m_led = new AddressableLED(Constants.LED_PORT);
//private AddressableLED m_led2 = new AddressableLED(Constants.LED_PORT2);
private LEDPattern green = LEDPattern.solid(Color.kGreen).atBrightness(Percent.of(100));
private LEDPattern red = LEDPattern.solid(Color.kRed).atBrightness(Percent.of(100));
private LEDPattern clear = LEDPattern.solid(Color.kWhite).atBrightness(Percent.of(100));
private LEDPattern white = LEDPattern.solid(Color.kWhite).atBrightness(Percent.of(10));
private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LED_LENGTH);
private AddressableLEDBufferView Mast = ledBuffer.createView(0, 33);
//private AddressableLEDBufferView Bar = ledBuffer.createView(0, 31);
private static final Distance ledSpacing = Meters.of(1 / 60.0);
private LEDPattern rainbow = LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), ledSpacing);
private LEDPattern coralblink = LEDPattern.rainbow(255, 128).blink(Seconds.of(1));
private LEDPattern greenwhite = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kWhite, Color.kGreen);
private LEDPattern greenscroll = greenwhite.scrollAtRelativeSpeed(Percent.per(Second).of(75));
private LEDPattern currentPattern;

    public Leds(){
		m_led.setLength(ledBuffer.getLength());
       // m_led2.setLength(ledBuffer2.getLength());
        currentPattern = rainbow;
        setDefaultCommand(this.animate());
		m_led.start();
        //m_led2.start();
    }

    public Command animate() {
        return run(() -> {
            currentPattern.applyTo(Mast);
            //green.applyTo(Bar);
        });
    }

    @Override
    public void periodic() {
        //setDefaultCommand(rainbow.applyTo(m_led));
        m_led.setData(ledBuffer);

    }

    public void setGreen(){
        currentPattern = green;
		green.applyTo(Mast);
    }

    public void setRed(){
        currentPattern = red;
		red.applyTo(Mast);
    }

    public void setRainbow() {
        currentPattern = rainbow;
        rainbow.applyTo(Mast);
    }

    public void coralbink(){
        currentPattern = coralblink;
        coralblink.applyTo(Mast);
    }

    public void greenscroll(){
        currentPattern = greenscroll;
        greenscroll.applyTo(Mast);
    }

    public void clear() {
        currentPattern = clear;
        clear.applyTo(Mast);
        //green.applyTo(Bar);
    }

    
}
