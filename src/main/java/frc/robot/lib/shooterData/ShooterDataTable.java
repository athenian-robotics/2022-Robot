package frc.robot.lib.shooterData;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.TreeMap;

public class ShooterDataTable implements Serializable {
    //private ShooterSpec[] dataTable;
    private TreeMap<Double, ShooterSpec> dataTable;
    public static final double MINDIST = 1.95;
    public static final double MAXDIST = 6.55;
    private static final double DINCREMENT = 0.5;
    private static final double K = 3.333334;
    private static final int LINEAR = 1;

    static final long serialVersionUID = 8520L;


    //Sets up the data table and pushes in empty specs
    public ShooterDataTable() {
        /*
        this.dataTable = new ShooterSpec[(int) Math.floor((MAXDIST - MINDIST) / DINCREMENT) + 1];

        for (int i = 0; i < dataTable.length; i++) {
            this.dataTable[i] = new ShooterSpec();
        }

         */
        this.dataTable = new TreeMap<>();
    }

    //adds new specs at specified index
    public void addSpecs(int index, double hoodAngle, double power){
        ShooterSpec newSpec = new ShooterSpec( hoodAngle, power);
        //dataTable[index] = newSpec;
    }

    //adds specs at a desired distance, rounded down to the nearest standard distance
    public void addSpecs(double distance, double hoodAngle, double power){
        ShooterSpec newSpec = new ShooterSpec(hoodAngle, power);
        dataTable.put(distance, newSpec);
        //int index = (int) ((distance - MINDIST) * K + 0.005);
        //dataTable[index] = newSpec;
    }

    //gets the desired specs for shooter at a distance, linearly interpolating between the
    //two closest data points.
    public ShooterSpec getSpecs(double distance){
        if(distance <= MINDIST || distance >= MAXDIST) return new ShooterSpec();
        if(dataTable.containsKey(distance)) return dataTable.get(distance);

        double prevDist = dataTable.lowerKey(distance);
        double nextDist = dataTable.higherKey(distance);
        ShooterSpec s1 = dataTable.get(prevDist);
        ShooterSpec s2 = dataTable.get(nextDist);
        double diffDist = nextDist - prevDist;
        double weight2 = (nextDist - distance) / diffDist;
        double weight1 = 1 - weight2;
        return new ShooterSpec(s1.getAngle() * weight1 + s2.getAngle() * weight2, s1.getPower() * weight1 + s2.getPower() * weight2);

    }


    //main method for testing
    public static void main(String[] args) {
        ShooterDataTable dt = new ShooterDataTable();
        dt.addSpecs(1.95, 21, 34.8);
        dt.addSpecs(2.11, 21.53, 35.4);
        dt.addSpecs(2.25, 22, 36);
        dt.addSpecs(2.40, 23, 36.4);
        dt.addSpecs(2.54, 23.7, 37);
        dt.addSpecs(2.85, 27.7, 37.2);
        dt.addSpecs(3.18, 28.5, 37.9);
        dt.addSpecs(3.41, 30.4, 38.5);
        dt.addSpecs(3.67, 32.5, 40.5);
        dt.addSpecs(3.96, 36, 43);
        dt.addSpecs(4.37, 36.8, 43.8);
        dt.addSpecs(4.67, 37.6, 44.65);
        dt.addSpecs(4.93, 39, 45.4);
        dt.addSpecs(5.31, 39.8, 46.9);
        dt.addSpecs(5.56, 40.5, 47.8);
        dt.addSpecs(5.85, 41, 49.5);
        dt.addSpecs(6.09, 41, 51.65);
        dt.addSpecs(6.42, 41, 52.7);
        dt.addSpecs(6.57, 41, 53.35);

        System.out.println(dt.getSpecs(6.5));
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

