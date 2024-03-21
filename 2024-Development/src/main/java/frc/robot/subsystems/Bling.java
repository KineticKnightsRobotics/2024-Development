package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.lib.Constants.BlingConstants;

public class Bling extends SubsystemBase {


    private static AddressableLED ledStrip;
    private static AddressableLEDBuffer ledStripBuffer;

    public Bling() {
        ledStrip = new AddressableLED(BlingConstants.ID_LED_STRIP);

        ledStripBuffer = new AddressableLEDBuffer(BlingConstants.ledStripLength);

        ledStrip.setLength(ledStripBuffer.getLength());

        ledStrip.setData(ledStripBuffer);

        ledStrip.start();
    }

    @Override
    public void periodic() {
        for (var i = 0; i < ledStripBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            ledStripBuffer.setRGB(i, 255, 0, 0);
        }
        ledStrip.setData(ledStripBuffer);
    }
    
}
