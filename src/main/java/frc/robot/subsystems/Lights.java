package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    private AddressableLEDBuffer mbuffer;
    private AddressableLED mLed;

    public Lights(int lenght, int port){
        this.mbuffer = new AddressableLEDBuffer(lenght);
        this.mLed = new AddressableLED(port);
        this.mLed.setLength(lenght);
        this.mLed.setData(mbuffer);
        this.mLed.setColorOrder(AddressableLED.ColorOrder.kRBG);
    }
    public void startLED(){
        mLed.start();
    }
    public void color(){
        // System.out.println("COLOR");
        mbuffer.createView(0, mbuffer.getLength()-1);
        LEDPattern pattern = LEDPattern.solid(Color.kRed);
        pattern.applyTo(mbuffer);
        mLed.setData(mbuffer);
    }
    @Override
    public void periodic(){
        color();
    }
}
