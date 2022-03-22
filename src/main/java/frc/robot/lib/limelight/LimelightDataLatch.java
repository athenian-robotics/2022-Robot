package frc.robot.lib.limelight;


//A LimelightDataLatch allows a command to put in a request for Limelight data, check if it has arrived, and retrieve
// it when it's ready.
//Throws a GoalNotFoundException when it times out without receiving data. timeoutSeconds is customizable but has a
// default value of a quarter second.
//Unlocked DataLatches (unlocked() == true) MUST BE RESET AND RESCHEDULED to retrieve new data. The simplest way is
// to call limelightSubsystem.addLatch(YOUR_LATCH.reset());
//I've left open() free of GoalNotFoundException because I trust you to always check unlocked() == true before
// opening the latch
public class LimelightDataLatch {
    public final LimelightDataType limelightDataType;
    private final long timeoutMillis;
    private long timeoutTimerStartMillis;
    private boolean unlocked;
    private boolean expired;
    private double value;

    public LimelightDataLatch(LimelightDataType limelightDataType) {
        this.unlocked = false;
        this.expired = false;
        this.limelightDataType = limelightDataType;
        this.timeoutMillis = 250; //default
        this.timeoutTimerStartMillis = System.currentTimeMillis();
    }

    public LimelightDataLatch(LimelightDataType limelightDataType, int timeoutSeconds) {
        this.unlocked = false;
        this.expired = false;
        this.limelightDataType = limelightDataType;
        this.timeoutMillis = 1000L * timeoutSeconds;
        this.timeoutTimerStartMillis = System.currentTimeMillis();
    }

    public void unlock(double value) {
        System.out.println(value);
        this.value = value;
        unlocked = true;
    }

    public boolean unlocked() throws GoalNotFoundException {
        if (System.currentTimeMillis() - timeoutTimerStartMillis > timeoutMillis) {
            expired = true;
            throw new GoalNotFoundException();
        } else {
            return unlocked;
        }
    }

    public boolean expired() {
        return expired;
    }

    public double open() {
        if (!unlocked)
            System.out.println("ERROR! Latch " + this + " for " + limelightDataType.toString() + " was opened before " +
                    "it was unlocked!");
        return value;
    }

    public LimelightDataLatch reset() {
        timeoutTimerStartMillis = System.currentTimeMillis();
        unlocked = false;
        expired = false;
        value = 0.0d;
        return this;
    }
}
