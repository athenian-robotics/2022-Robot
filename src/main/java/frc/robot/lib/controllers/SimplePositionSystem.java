package frc.robot.lib.controllers;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
 * Defines a simple velocity system using a kalman filter and a linear quadratic regulator
 */
public class SimplePositionSystem {
    private double kS;

    private double maxControlEffort; // volts
    private double modelStandardDeviation;
    private double encoderStandardDeviation;

    private double filteredVelocity;

    private LinearSystem<N2, N1, N1> system;
    private LinearQuadraticRegulator<N2, N1, N1> regulator;
    private KalmanFilter<N2, N1, N1> filter;
    private LinearSystemLoop<N2, N1, N1> loop;

    public SimplePositionSystem(double kS, double kV, double kA, double maxError, double maxcontroleffort, double modelstandarddev, double encoderstandarddev, double looptime) {
        this.kS = kS;
        maxControlEffort = maxcontroleffort;
        modelStandardDeviation = modelstandarddev;
        encoderStandardDeviation = encoderstandarddev;

        system = LinearSystemId.identifyPositionSystem(kV, kA);
        regulator = new LinearQuadraticRegulator<>(system, VecBuilder.fill(maxError, maxError), VecBuilder.fill(maxControlEffort), looptime);
        filter = new KalmanFilter<>(Nat.N2(), Nat.N1(), system, VecBuilder.fill(modelStandardDeviation, modelStandardDeviation), VecBuilder.fill(encoderStandardDeviation), looptime);
        loop = new LinearSystemLoop<>(system, regulator, filter, maxControlEffort - kS, looptime);
    }

    /**
     *
     * @param output The desired output of the system
     */
    public void set(double output) {
        loop.setNextR(output,0); // set output
    }

    /**
     *
     * @param current The measured velocity by the encoder
     */
    public void update(double current) {
        loop.correct(VecBuilder.fill(current));
        filteredVelocity = loop.getXHat(0);
        loop.predict(0.02);
    }

    /**
     *
     * @return The percent output that the controller should run at
     */
    public double getOutput() {
        return (loop.getU(0) - kS*Math.signum(loop.getNextR(0))) / maxControlEffort;
    }

    /**
     * Get the filtered velocity of the system
     *
     */
    public double getVelocity() {
        return filteredVelocity;//loop.getXHat(0);
    }

    public LinearSystem<N2, N1, N1> getSystem() {
        return system;
    }

    public LinearQuadraticRegulator<N2, N1, N1> getLQR() {
        return regulator;
    }

    public KalmanFilter<N2, N1, N1> getKalmanFiter() {
        return filter;
    }

    public LinearSystemLoop<N2, N1, N1> getLinearSystemLoop() {
        return loop;
    }
}