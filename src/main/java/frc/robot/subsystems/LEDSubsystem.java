package frc.robot.subsystems;

import static frc.robot.Constants.LEDConstants.LEDPort;
import static frc.robot.Constants.LEDConstants.LEDStripLength;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLEDBuffer ledStripBuffer;
  private final AddressableLED ledStrip;
  private boolean continuousOutputEnabled;

  public LEDSubsystem() {
    ledStripBuffer = new AddressableLEDBuffer(LEDStripLength);
    ledStrip = new AddressableLED(LEDPort);

    ledStrip.setLength(ledStripBuffer.getLength());
    ledStrip.start();
  }

  public void setAllColor(int R, int G, int B) {
    for (int i = 0; i < ledStripBuffer.getLength(); i++) {
      ledStripBuffer.setRGB(i, R, G, B);
    }
    ledStrip.setData(ledStripBuffer);
  }

  public void setColorAtIndex(int i, int R, int G, int B) {
    if (i > 0 && i < ledStripBuffer.getLength()) ledStripBuffer.setRGB(i, R, G, B);
  }

  public void enableContinuousOutput() {
    continuousOutputEnabled = true;
  }

  public void disableContinuousOutput() {
    continuousOutputEnabled = false;
  }

  public void update() {
    ledStrip.setData(ledStripBuffer);
  }

  public void disable() {
    continuousOutputEnabled = false;
  }

  public void periodic() {
    // if (continuousOutputEnabled) update();
  }
}
