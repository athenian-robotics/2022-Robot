package frc.robot.lib.shooterData;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.io.Serializable;

public class ShooterDataTable implements Serializable {
    private ShooterSpec[] dataTable;
    private static final double MINDIST = 1.95;
    private static final double MAXDIST = 6.15;
    private static final double DINCREMENT = 0.3;
    private static final double K = 3.333334;
    private static final int LINEAR = 1;

    //Sets up the data table and pushes in empty specs
    public ShooterDataTable() {
        this.dataTable = new ShooterSpec[(int) Math.floor((MAXDIST - MINDIST) / DINCREMENT) + 1];
        for (int i = 0; i < dataTable.length; i++) {
            this.dataTable[i] = new ShooterSpec();
        }
    }

    //adds new specs at specified index
    public void addSpecs(int index, double speed, double hoodAngle, double power){
        ShooterSpec newSpec = new ShooterSpec(speed, hoodAngle, power);
        dataTable[index] = newSpec;
    }

    //adds specs at a desired distance, rounded down to the nearest standard distance
    public void addSpecs(double distance, double speed, double hoodAngle, double power){
        ShooterSpec newSpec = new ShooterSpec(speed, hoodAngle, power);
        int index = (int) ((distance - MINDIST) * K + 0.005);
        dataTable[index] = newSpec;
    }

    //gets the desired specs for shooter at a distance, linearly interpolating between the
    //two closest data points.

    public ShooterSpec getSpecs(double distance){
        if(distance < MINDIST || distance >= MAXDIST)	return new ShooterSpec();
        int index = (int) ((distance - MINDIST) * K + 0.005);
        System.out.println("INDEX = " + index);
        ShooterSpec s1 = dataTable[index];
        ShooterSpec s2 = dataTable[index+1];
        double weight2 = (distance - MINDIST - (DINCREMENT * index)) / DINCREMENT;
        double weight1 = 1 - weight2;
        return new ShooterSpec(s1.getSpeed() * weight1 + s2.getSpeed() * weight2, s1.getAngle() * weight1 + s2.getAngle() * weight2, s1.getPower() * weight1 + s2.getPower() * weight2);
    }

    //main method for testing
    public static void main(String[] args) {
        ShooterDataTable dt = new ShooterDataTable();
        //dt.addSpecs(2.7, 0, 28, 35);
        dt.addSpecs(1.95, 0, 21, 35);
        //dt.addSpecs(2.11, 0, 20, 35);
        dt.addSpecs(2.25, 0, 22, 36);
        //dt.addSpecs(2.40, 0, 23, 37);
        dt.addSpecs(2.55, 0, 23, 37);
        dt.addSpecs(2.85, 0, 24, 37.5);
        dt.addSpecs(3.15, 0, 25, 39);
        dt.addSpecs(3.45, 0, 25, 40.8);
        dt.addSpecs(3.75, 0, 26.5, 41.5);
        dt.addSpecs(4.05, 0, 27.5, 42.2);
        dt.addSpecs(4.35, 0, 28.5, 43);
        dt.addSpecs(4.65, 0, 29.5, 44);
        dt.addSpecs(4.95, 0, 30.5, 45.2);
        dt.addSpecs(5.25, 0, 31.3, 46.8);
        dt.addSpecs(5.55, 0, 31.3, 46.8);
        dt.addSpecs(5.85, 0, 33, 47);

        try {
            FileOutputStream fileOut = new FileOutputStream("src/main/deploy/dt.ser");
            System.out.println("built file output stream");
            ObjectOutputStream out = new ObjectOutputStream(fileOut);
            System.out.println("Ready to write object");
            out.writeObject(dt);
            System.out.println("Object written");
            out.flush();

            out.close();
        }catch(IOException e){
            System.out.println(e + " encountered. " + "Wow, such empty");
        }
    }

}

