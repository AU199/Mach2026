package frc.robot.subsystems.Simulation;
import java.util.function.ToDoubleBiFunction;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Simulation.helperClasses.*;
import frc.robot.subsystems.Simulation.NewtonRapson;
public class Simulation {
    Pose3d hubPosition;
    int iterations = 0;
    double CD;
    double CL;
    Translation3d omega;
    public static NewtonRapson newtonRapsonSimulator;
    public Simulation(double CD, double CL, Pose3d hubPose){
        this.hubPosition = hubPose;
        Simulation.newtonRapsonSimulator = new NewtonRapson(CD, CL, hubPose);
    }
    /**
     * @param initalVelocity The inital velocity of the fuel
     * @param initialOmega The intial angular velocity of the fuel (rads/second)
     * @param initalPosition The inital position of the fuel with respect to the
     *                       field origin
     * @param initalTheta    the initial launch angle of the shooter
     * @param epsilon        the tiny change in theta and phi when calculating the
     *                       derivitive
     * @param iterationCount the maximum amount of times that this function can run
     * @param dt             the time step of the simulation
     * @param tolerance      the tolerance allowed for the simulation
     * @return the new theta and phi for a better shot
     */
    public double[] findThetaPhi(double initalVelocity, double initalOmega,Pose3d initalPosition,
            double initalTheta, Float epsilon,
            int iterationCount, double dt, double tolerance) {
        // System.out.println(hubPosition);
        double initialPhi = initalPosition.getRotation().getZ();
        velocity velocity = new velocity(initalVelocity, initalTheta, initialPhi);
        System.out.println(velocity);
        Transform3d positionDifference = initalPosition.minus(hubPosition);
        System.out.println(hubPosition);
        double theta = initalTheta;
        double phi = initialPhi;
        double Diff = Math.sqrt(Math.pow(positionDifference.getX(), 2) + Math.pow(positionDifference.getY(), 2));
        this.iterations = 0;
        while (iterations <= iterationCount) {
            double horizontalSpeed = Math.sqrt(Math.pow(velocity.getX(), 2) + Math.pow(velocity.getY(), 2));
            double maxTime = (Diff / Math.max(horizontalSpeed, 0.1));
            Translation3d omega = new Translation3d(initalOmega,new Rotation3d(0,theta,phi));
            double[] deltas = newtonRapsonSimulator.NewtonRappingSon(velocity, omega,initalPosition, epsilon, dt, maxTime, hubPosition);
            if (Math.hypot(deltas[2], deltas[3]) < tolerance) {
                break;
            }
            velocity = velocity.changePhi(deltas[1]).changeTheta(deltas[0]);
            System.out.println(iterations + " " + velocity);
            this.iterations++;

        }
        SmartDashboard.putNumber("ShooterThetaSIMULATION", theta);
        double[] thetaPhi = { velocity.theta, velocity.phi };
        return thetaPhi;
    }
}
