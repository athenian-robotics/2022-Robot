package frc.robot.lib.shooterData;
import java.io.Serializable;

//Stores launch speed, hood angle, and power as doubles
public class ShooterSpec implements Serializable{
    private double speed;
    private double hoodAngle;
    private double power;

    public ShooterSpec() {
        this.speed = 0.0;
        this.hoodAngle = 0.0;
        this.power = 0.0;
    }

    public ShooterSpec(double theta, double power) {
        this.speed = 0.0;
        this.hoodAngle = theta;
        this.power = power;
    }

    // power is constrained between 0 and 1, hoodAngle is constrained between 9 and 40 deg
    public ShooterSpec(double v0, double theta, double power) {
        this.speed = v0;
        this.hoodAngle = theta > 40 ? 40 : (theta < 9 ? 9 : theta);
        this.power = power > 1 ? 1 : (power < 0 ? 0 : power);
    }

    public double getSpeed() {
        return this.speed;
    }

    public double getAngle() {
        return this.hoodAngle;
    }

    public double getPower() {
        return this.power;
    }

    public String toString() {
        return ("speed = " + speed + " m/s \n" + "hoodAngle = " + hoodAngle + " degrees \n" + " power = " + power + "\n");
    }
}
