package frc.robot.subsystems.Simulation;

import java.time.Year;
import java.util.Optional;

import javax.imageio.stream.IIOByteBuffer;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.Simulation.helperClasses.*;
import frc.robot.util.FuelSim.Hub;

public class RK4 {
    double rho = 1.2;
    double CD;
    double CL;
    double mass = 0.203;
    double radius = 0.075;
    Translation3d omega;
    double A = Math.PI * Math.pow(radius, 2);
    double hubRadius;
    private Translation3d gravity = new Translation3d(0, 0, -9.81);

    public RK4(double CD, double CL, double hubRadius){
        this.CD = CD;
        this.CL = CL;
        this.hubRadius = hubRadius;
    }
    
    
    
    /** 
     * @param currVelocity the current velocity of the ball
     * @return The acceleration vector
     */
    public Translation3d acceleration(Translation3d currVelocity, Translation3d omega) {
        if (currVelocity.getNorm() < 1e-5) {
        return gravity;
        }
        double DragMag = (0.5 * rho * CD * A * Math.pow(currVelocity.getNorm(), 2)) / mass;
        Translation3d DragAccel = currVelocity.times(DragMag / currVelocity.getNorm());
        double magnusCoeff = (0.5 * rho * CL * A * radius) / mass;
        Translation3d MagnusAccel = new Translation3d(omega.cross(currVelocity)).times(magnusCoeff);
        return gravity.plus(MagnusAccel).plus(DragAccel);
    }

    /**
     * This method RK4 in order to calculate the next step in the trajectory of the
     * ball.
     * 
     * @param velocity the inital velocity before the state update
     * @param position the inital position before the state update
     * @param dt       the time step the rk4 shouldl take
     * @return returns the new position and velocity in a state
     */
    public state rk4Step(Translation3d velocity, Translation3d omega,Translation3d position, double dt) {
        Translation3d tempVel = velocity;
        Translation3d tempPos = position;

        Translation3d k1_v = acceleration(tempVel, omega);
        Translation3d k1_x = tempVel;

        Translation3d v2 = tempVel.plus(k1_v.times(dt / 2));
        Translation3d x2 = tempPos.plus(k1_x.times(dt / 2));
        Translation3d k2_v = acceleration(v2, omega);
        Translation3d k2_x = v2;

        Translation3d v3 = tempVel.plus(k2_v.times(dt / 2));
        Translation3d x3 = tempPos.plus(k2_x.times(dt / 2));
        Translation3d k3_v = acceleration(v3, omega);
        Translation3d k3_x = v3;

        Translation3d v4 = tempVel.plus(k3_v.times(dt));
        Translation3d x4 = tempPos.plus(k3_x.times(dt));
        Translation3d k4_v = acceleration(v4, omega);
        Translation3d k4_x = v4;

        Translation3d newPosition = position
                .plus((k1_x.plus(k2_x.times(2)).plus(k3_x.times(2)).plus(k4_x)).times(dt / 6));
        Translation3d newVelocity = velocity
                .plus((k1_v.plus(k2_v.times(2)).plus(k3_v.times(2)).plus(k4_v)).times(dt / 6));

        return new state(newVelocity, newPosition);
    }

    /**
     * Runs RK4 states in order calculate the x error and y error of the current
     * shot and phi angle.
     * 
     * @param initalVelocity The initial velocity of the fuel
     * @param initalPosition The initial postion of the fuel in respect of the
     *                       origin (located at the bottom left of the field aka.
     *                       the left corner of th
     *                       blue alliance)
     * @param dt             The tiny step in time that the simulation should take
     * @param maxTime        The maximum time that the simulation can last
     * @param hubPosition    The position of the hub in respect of the origin
     *                       (located at the bottom left of the field aka. the left
     *                       corner of th
     *                       blue alliance)
     * @return the x error and y error FORMAT [xError, yError]
     */
    public double[] getXandYError(velocity initalVelocity, Translation3d omega,Pose3d initalPosition, double dt, double maxTime,
            Pose3d hubPosition) {
        Translation3d pos = new Translation3d(initalPosition.getX(), initalPosition.getY(), initalPosition.getZ());
        Translation3d vel = new Translation3d(initalVelocity.getX(), initalVelocity.getY(), initalVelocity.getZ());
        // System.out.println(vel.getZ());
        double time = 0;
        while (true) {
            double horizontalDistance = pos.toTranslation2d().getDistance(hubPosition.toPose2d().getTranslation());
            // System.out.println("Horizontal distance "+ horizontalDistance);
            if (horizontalDistance < hubRadius && pos.getZ() <= hubPosition.getZ()) {
                break;
            }

            if (time > maxTime) {
                break;
            }

            state rk4State = rk4Step(vel, omega, pos, dt);
            vel = rk4State.velocity;
            pos = rk4State.position;
            if (Double.isNaN(pos.getZ())) {
                System.exit(0);
            }
            time += dt;
        }
        // System.out.println(hubPosition);
        double xError = pos.getX() - hubPosition.getX();
        double yError = pos.getY() - hubPosition.getY();
        // System.out.println("X" + this.hubPosition.getX() + " " + pos.getX());
        // System.out.println("Y" + this.hubPosition.getY() + " " + pos.getY());

        double[] errors = { xError, yError };
        // System.out.println("X ERROR" + errors[0]);
        // System.out.println("Y ERROR" + errors[1]);

        SmartDashboard.putNumberArray("XYError", errors);

        return errors;

    }        
    
    
    /**
     * @param a The first Translation3d object
     * @param b The second Translation3d object
     * @return The cross product of the two Translation3d objects
     */
    Translation3d crossProduct(Translation3d a, Translation3d b) {
        return new Translation3d(
                a.getY() * b.getZ() - a.getZ() * b.getY(), // x component
                a.getZ() * b.getX() - a.getX() * b.getZ(), // y component
                a.getX() * b.getY() - a.getY() * b.getX() // z component
        );
    }
}
