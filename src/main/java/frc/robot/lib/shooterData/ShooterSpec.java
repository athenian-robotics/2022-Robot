package frc.robot.lib.shooterData;

import java.io.Serializable;

//Stores launch speed, hood angle, and power as doubles
public class ShooterSpec implements Serializable {
    static final long serialVersionUID = 8521L;
    private final double hoodAngle;
    private final double power;

    public ShooterSpec() {
        this.hoodAngle = 14.0;
        this.power = 33.5;
    }

    public ShooterSpec(double theta, double power) {
        this.hoodAngle = theta;
        this.power = power;
    }

    public double getAngle() {
        return this.hoodAngle;
    }

    public double getPower() {
        return this.power;
    }

    public String toString() {
        return ("hoodAngle = " + hoodAngle + " degrees \n" + " power = " + power + "\n");
    }
}
