package frc.robot.lib.shooterData;
import java.io.*;

public class ShooterDataTable implements Serializable{
    private ShooterSpec[] dataTable;
    private static final double MINDIST = 1.0;
    private static final double MAXDIST = 7.0;
    private static final double DINCREMENT = 0.2;
    private static final int K = 5;
    private static final int LINEAR = 1;

    //Sets up the data table and pushes in empty specs
    public ShooterDataTable(){
        this.dataTable =new ShooterSpec[(int) Math.floor((MAXDIST - MINDIST) / DINCREMENT) + 1];
        for(int i = 0; i < dataTable.length; i++){
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
    public static void main(String[] args){
        ShooterDataTable dt = new ShooterDataTable();
        dt.addSpecs(1.0, 6.6158354995, 10.03334, 0.6);
        dt.addSpecs(1.2, 6.741745, 11.3128, 0.62);
        dt.addSpecs(1.4, 6.872057383, 12.4430, 0.65);
        dt.addSpecs(1.6, 7.0050826, 13.4479, 0.67);
        dt.addSpecs(6.8, 10.1950, 23.3826, 0.95);
        dt.addSpecs(7.0, 10.3302, 23.5712, 0.97);
        System.out.println(dt.getSpecs(1.0));
        System.out.println(dt.getSpecs(1.1) + "" + dt.getSpecs(6.9) + "" + dt.getSpecs(6.99));

        try{
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

