package frc.robot.lib.colorwheel;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.lib.colorwheel.WheelColors.*;

public class ColorWheelUtils {
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    private boolean isYellow;
    private boolean isGreen;
    private boolean isBlue;
    private boolean isRed;

    public ColorWheelUtils() {}

    private void updateString(String currentColor) {SmartDashboard.putString("Current Color", currentColor);}

    private void resetIsColorBooleans() {
        isRed = false;
        isBlue = false;
        isGreen = false;
        isYellow = false;
    }

    public WheelColors currentColor() {
        WheelColors currentColor;
        Color detectedColor = colorSensor.getColor();
        SmartDashboard.putNumber("red", detectedColor.red);
        SmartDashboard.putNumber("blue", detectedColor.blue);
        SmartDashboard.putNumber("green", detectedColor.green);
        double checkColorRed = detectedColor.red * 255;
        double checkColorGreen = detectedColor.green * 255;
        double checkColorBlue = detectedColor.blue * 255;

        //blue is weighted higher because blue balls have red .2 and blue .3 and red balls have red .4 and blue .2
        currentColor = checkColorRed > checkColorBlue*1.3? RED : BLUE;
        resetIsColorBooleans();
        updateString(currentColor.name());
        return currentColor;
    }

    public double currentProximity() {return colorSensor.getProximity();}
}
