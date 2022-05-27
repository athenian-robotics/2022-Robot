package frc.robot.lib.shooterData;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.TreeMap;

public class ShooterDataTable implements Serializable {
  private static final double MINDIST = 1.94;
  private static final double MAXDIST = 8;
  private static final double shooterDataTableScalar =
      1; // we had to change the limelight code and scale the distances to what they used to be
  // private ShooterSpec[] dataTable;
  private final TreeMap<Double, ShooterSpec> dataTable;

  // Sets up the data table and pushes in empty specs
  public ShooterDataTable() {
    this.dataTable = new TreeMap<>();
  }

  // main method for testing
  public static void main(String[] args) {
    ShooterDataTable dt = new ShooterDataTable();
    dt.addSpecs(1.95, 21, 34.8, 0.1);
    dt.addSpecs(2.11, 21.53, 35.4, 0.1);
    dt.addSpecs(2.25, 22, 36, 0.1);
    dt.addSpecs(2.40, 23, 36.4, 0.1);
    dt.addSpecs(2.54, 23.7, 37, 0.1);
    dt.addSpecs(2.85, 27.7, 37.2, 0.1);
    dt.addSpecs(3.18, 28.5, 37.9, 0.1);
    dt.addSpecs(3.41, 30.4, 38.5, 0.1);
    dt.addSpecs(3.67, 32.5, 40.5, 0.1);
    dt.addSpecs(3.96, 36, 43, 0.1);
    dt.addSpecs(4.37, 36.8, 43.8, 0.1);
    dt.addSpecs(4.67, 37.6, 44.65, 0.1);
    dt.addSpecs(4.93, 39, 45.4, 0.1);
    dt.addSpecs(5.31, 39.8, 46.9, 0.1);
    dt.addSpecs(5.56, 40.5, 47.8, 0.1);
    dt.addSpecs(5.85, 41, 49.5, 0.1);
    dt.addSpecs(6.09, 41, 51.65, 0.1);
    dt.addSpecs(6.42, 41, 52.7, 0.1);
    dt.addSpecs(6.57, 41, 53.35, 0.1);
    dt.addSpecs(1.75, 26, 41, 0.01);
    dt.addSpecs(3.29, 30, 43, 0.01);
    dt.addSpecs(3.51, 31, 37.2, 0.01);
    dt.addSpecs(3.76, 35, 38.2, 0.01);
    dt.addSpecs(4.01, 38, 40.9, 0.01);
    dt.addSpecs(4.27, 38.13, 40.96, 0.01);
    dt.addSpecs(4.56, 35.5, 45, 0.01);
    dt.addSpecs(4.82, 40.5, 41.28, 0.01);
    dt.addSpecs(5.00, 41, 42.3, 0.01);
    dt.addSpecs(6.5, 41, 55, 0.01);
    for (double i = 5.1; i < 8; i += 0.1) {
      dt.addSpecs(i, 41, 72.4 * Math.log(i) - 8.23734, 0.01);
    } // evil hack

    try {
      FileOutputStream fileOut = new FileOutputStream("src/main/deploy/dt.ser");
      System.out.println("built file output stream");
      ObjectOutputStream out = new ObjectOutputStream(fileOut);
      System.out.println("Ready to write object");
      out.writeObject(dt);
      System.out.println("Object written");
      out.flush();

      out.close();
    } catch (IOException e) {
      System.out.println(e + " encountered. " + "Wow, such empty");
    }
  }

  // adds specs at a desired distance, rounded down to the nearest standard distance
  public void addSpecs(double distance, double hoodAngle, double power, double TOF) {
    ShooterSpec newSpec = new ShooterSpec(hoodAngle, power, TOF);
    dataTable.put(distance, newSpec);
  }

  // gets the desired specs for shooter at a distance, linearly interpolating between the
  // two closest data points.
  public ShooterSpec getSpecs(double distance) {
    distance *= shooterDataTableScalar;
    if (distance <= MINDIST || distance >= MAXDIST) return new ShooterSpec();
    if (dataTable.containsKey(distance)) return dataTable.get(distance);

    double prevDist = dataTable.lowerKey(distance);
    double nextDist = dataTable.higherKey(distance);
    ShooterSpec s1 = dataTable.get(prevDist);
    ShooterSpec s2 = dataTable.get(nextDist);
    double diffDist = nextDist - prevDist;
    double weight2 = (nextDist - distance) / diffDist;
    double weight1 = 1 - weight2;
    return new ShooterSpec(
        s1.getAngle() * weight1 + s2.getAngle() * weight2,
        s1.getPower() * weight1 + s2.getPower() * weight2,
        s1.getTOF() * weight1 + s2.getTOF() * weight2);
  }
}
