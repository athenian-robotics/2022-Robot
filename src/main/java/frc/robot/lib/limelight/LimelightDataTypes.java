package frc.robot.lib.limelight;


interface ValidityComparator {
    boolean checkValidity(double lastValue, double currentValue);
}

public enum LimelightDataTypes {
    DISTANCE(0, (double lastValue, double currentValue) -> {return currentValue <= 6 && currentValue >= 1.75;}),
    HORIZONTAL_OFFSET(1, (double lastValue, double currentValue) -> {return Math.abs(Math.abs(lastValue) - Math.abs(currentValue)) < 5;});

    public final int llpythonIndex;
    public final ValidityComparator validityComparator;

    LimelightDataTypes(int llpythonIndex, ValidityComparator validityComparator) {
        this.validityComparator = validityComparator;
        this.llpythonIndex = llpythonIndex;
    }

    public boolean isValid(double lastValue, double currentValue) {
        return validityComparator.checkValidity(lastValue, currentValue);
    }
}
