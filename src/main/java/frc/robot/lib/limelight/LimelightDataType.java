package frc.robot.lib.limelight;

// Holds the indexes of named Limelight values. Capable of holding data validity and comparison
// functions as well.
public enum LimelightDataType {
  DISTANCE(0),
  HORIZONTAL_OFFSET(1);

  public final int llpythonIndex;

  LimelightDataType(int llpythonIndex) {
    this.llpythonIndex = llpythonIndex;
  }
}
