package frc.robot.lib.limelight;

public class LimelightDataLatch {
    public final LimelightDataTypes dataType;
    private final double timeoutMilliseconds;
    private final double timeoutTimerStart;

    private boolean unlocked;
    private double value;

    public LimelightDataLatch(LimelightDataTypes dataType) {
        this.unlocked = false;
        this.dataType = dataType;
        this.timeoutMilliseconds = 250;
        this.timeoutTimerStart = System.currentTimeMillis();
    }

    public LimelightDataLatch(LimelightDataTypes dataType, double timeoutSeconds) {
        this.unlocked = false;
        this.dataType = dataType;
        this.timeoutMilliseconds = 1000*timeoutSeconds;
        this.timeoutTimerStart = System.currentTimeMillis();
    }

    public void unlock(double value) {
        this.unlocked = true;
        this.value = value;
    }

    public boolean test() throws GoalNotFoundException {
        if (System.currentTimeMillis() - timeoutTimerStart > timeoutMilliseconds) throw new GoalNotFoundException();
        return unlocked;
    }

    public double open() throws GoalNotFoundException {
        if (unlocked) return value;
        else throw new GoalNotFoundException();
    }
}
