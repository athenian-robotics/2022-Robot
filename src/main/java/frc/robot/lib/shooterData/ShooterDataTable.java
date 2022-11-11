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
    dt.addSpecs(1, 25, 30, 0.1);
    // dt.addSpecs(1.5, );
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
