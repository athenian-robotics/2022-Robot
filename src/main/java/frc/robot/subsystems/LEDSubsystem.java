package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LEDConstants.LEDPort;
import static frc.robot.Constants.LEDConstants.LEDStripLength;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLEDBuffer ledStripBuffer = new AddressableLEDBuffer(LEDStripLength);

    public LEDSubsystem() {
        AddressableLED ledStrip = new AddressableLED(LEDPort);
        ledStrip.setLength(ledStripBuffer.getLength());
        ledStrip.setData(ledStripBuffer);
        ledStrip.start();
    }

    public void setAllColor(int R, int G, int B) {
        for (int i = 0; i < ledStripBuffer.getLength(); i++) {
            ledStripBuffer.setRGB(i, R, G, B);
        }
    }

    public void SetLeftHalfColor(int R, int G, int B) {
        for (int i = 0; i < ledStripBuffer.getLength() / 2; i++) {
            ledStripBuffer.setRGB(i, R, G, B);
        }
    }

    public void SetRightHalfColor(int R, int G, int B) {
        for (int i = ledStripBuffer.getLength() / 2; i < ledStripBuffer.getLength(); i++) {
            ledStripBuffer.setRGB(i, R, G, B);
        }
    }
}

