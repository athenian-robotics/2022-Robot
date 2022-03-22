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
        double checkColorRed = detectedColor.red * 255;
        double checkColorGreen = detectedColor.green * 255;
        double checkColorBlue = detectedColor.blue * 255;

        //Range for RED
        if ((checkColorRed >= checkColorGreen + 30) && (checkColorRed >= checkColorBlue + 30)) {
            updateString("RED");
            currentColor = RED;
            isRed = true;
        }
        //Range for BLUE
        else if ((checkColorBlue >= checkColorGreen) && (checkColorBlue - 60 <= checkColorGreen)
                || (checkColorBlue <= checkColorGreen) && (checkColorBlue + 60 >= checkColorGreen)) {
            updateString("BLUE");
            currentColor = BLUE;
            isBlue = true;
        }
        //Range for YELLOW
        else if ((checkColorRed >= 90) && (checkColorRed <= checkColorGreen) && (checkColorRed + 20 >= checkColorGreen)) {
            updateString("YELLOW");
            currentColor = YELLOW;
            isYellow = true;
        }
        //Assume GREEN
        else {
            updateString("GREEN");
            currentColor = GREEN;
            isGreen = true;
        }
        //Update the color value,
        updateColorsOnDashboard();
        resetIsColorBooleans();
        return currentColor;
    }

    public void updateColorsOnDashboard() {
        //Update boolean boxes for each color for visual representation
        SmartDashboard.putBoolean("RED", isRed);
        SmartDashboard.putBoolean("BLUE", isBlue);
        SmartDashboard.putBoolean("YELLOW", isYellow);
        SmartDashboard.putBoolean("GREEN", isGreen);
    }

    public double currentProximity() {return colorSensor.getProximity();}
}
