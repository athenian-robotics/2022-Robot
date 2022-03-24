package frc.robot.lib.controllers;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
 * Defines a simple velocity system using a kalman filter and a linear quadratic regulator
 */
public class SimpleVelocitySystem {
    private final double kS;

    private final double maxControlEffort; // volts

    private double filteredVelocity;

    private final LinearSystem<N1, N1, N1> system;
    private final LinearQuadraticRegulator<N1, N1, N1> regulator;
    private final KalmanFilter<N1, N1, N1> filter;
    private final LinearSystemLoop<N1, N1, N1> loop;

    public SimpleVelocitySystem(double kS, double kV, double kA, double maxError, double maxcontroleffort,
                                double modelstandarddev, double encoderstandarddev, double looptime) {
        this.kS = kS;
        maxControlEffort = maxcontroleffort;

        system = LinearSystemId.identifyVelocitySystem(kV, kA);
        regulator = new LinearQuadraticRegulator<>(system, VecBuilder.fill(maxError),
                VecBuilder.fill(maxControlEffort), looptime);
        filter = new KalmanFilter<>(Nat.N1(), Nat.N1(), system, VecBuilder.fill(modelstandarddev),
                VecBuilder.fill(encoderstandarddev), looptime);
        loop = new LinearSystemLoop<>(system, regulator, filter, maxControlEffort, looptime);
    }

    /**
     * @param output The desired output of the system
     */
    public void set(double output) {
        loop.setNextR(VecBuilder.fill(output)); // set output
    }

    /**
     * @param current The measured velocity by the encoder
     */
    public void update(double current) {
        loop.correct(VecBuilder.fill(current));
        filteredVelocity = loop.getXHat(0);
        loop.predict(0.02);
    }

    /**
     * @return The percent output that the controller should run at
     */
    public double getOutput() {
        return (loop.getU(0) - kS * Math.signum(loop.getNextR(0))) / maxControlEffort;
    }

    /**
     * Get the filtered velocity of the system
     */
    public double getVelocity() {
        return filteredVelocity;//loop.getXHat(0);
    }

    public LinearSystem<N1, N1, N1> getSystem() {
        return system;
    }

    public LinearQuadraticRegulator<N1, N1, N1> getLQR() {
        return regulator;
    }

    public KalmanFilter<N1, N1, N1> getKalmanFiter() {
        return filter;
    }

    public LinearSystemLoop<N1, N1, N1> getLinearSystemLoop() {
        return loop;
    }
}