package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Simulation {
    // Mass of the ball
    private double mass = 0.203;
    private Translation3d gravity = new Translation3d(0, 0, -9.81);
    private double CD;
    private double CL;
    // Air density kg/m^3
    private double rho = 1.2;
    // radius of the ball
    private double radius = 0.075;
    private Translation3d omega;
    private Translation3d velocity;
    private Optional<Alliance> currentAlliance = DriverStation.getAlliance();
    private Pose3d hubPosition = (currentAlliance.get() == Alliance.Blue)
            ? new Pose3d(InchtoMeter(182.11), InchtoMeter(158.84), InchtoMeter(72.0), new Rotation3d(0, 0, 0))
            : new Pose3d(InchtoMeter(651.22 - 182.11), InchtoMeter(158.84), InchtoMeter(72.0), new Rotation3d(0, 0, 0));

    double A = Math.PI * Math.pow(radius, 2);

    public Simulation(double CD, double CL, Translation3d Omega, Translation3d velocity) {
        this.CD = CD;
        this.CL = CL;
        this.omega = Omega;
        this.velocity = velocity;
    }

    private Translation3d acceleration(Translation3d currVelocity) {
        double DragMag = (0.5 * rho * CD * A * Math.pow(currVelocity.getNorm(), 2)) / mass;
        Translation3d DragAccel = currVelocity.times(-DragMag / currVelocity.getNorm());
        double magnusCoeff = (0.5 * rho * CL * A * radius) / mass;
        Translation3d MagnusAccel = crossProduct(omega, currVelocity).times(magnusCoeff);
        return gravity.plus(DragAccel).plus(MagnusAccel);
    }

    private state rk4Step(Translation3d velocity, Translation3d position, double dt) {
        Translation3d k1_v = acceleration(velocity);
        Translation3d k1_x = velocity;

        Translation3d v2 = velocity.plus(k1_v.times(dt / 2));
        Translation3d x2 = position.plus(k1_x.times(dt / 2));
        Translation3d k2_v = acceleration(v2);
        Translation3d k2_x = v2;

        Translation3d v3 = v2.plus(k2_v.times(dt / 2));
        Translation3d x3 = x2.plus(k2_x.times(dt / 2));
        Translation3d k3_v = acceleration(v3);
        Translation3d k3_x = v3;

        Translation3d v4 = v3.plus(k3_v.times(dt / 2));
        Translation3d x4 = x3.plus(k3_x.times(dt / 2));
        Translation3d k4_v = acceleration(v4);
        Translation3d k4_x = v4;

        Translation3d newPosition = position.plus(k1_x.plus(k2_x.times(2)).plus(k3_x.times(2)).plus(k4_x));
        Translation3d newVelocity = velocity.plus(k1_v.plus(k2_v.times(2)).plus(k3_v.times(2)).plus(k4_v));

        return new state(newVelocity, newPosition);
    }

    public double[] Simulate(velocity initalVelocity, Pose3d initalPosition, double dt, double maxTime, double theta) {
        Translation3d pos = new Translation3d(initalPosition.getX(), initalPosition.getY(), initalPosition.getZ());
        Translation3d vel = new Translation3d(initalVelocity.getX(), initalVelocity.getY(), initalVelocity.getZ());
        double time = 0;
        int Tick = 0;
        while ((Tick > 0 && pos.getZ() <= hubPosition.getZ()) && time < maxTime) {
            state rk4State = rk4Step(vel, pos, dt);
            vel = rk4State.velocity;
            pos = rk4State.position;
            if (pos.getZ() >= hubPosition.getZ() && Tick == 0) {
                Tick = 1;
            }
            time += dt;
        }

        double xError = pos.getX() - hubPosition.getX();
        double yError = pos.getY() - hubPosition.getY();
        double[] errors = { xError, yError };

        return errors;

    }

    public double[] NewtonRappingSon(velocity velocity, double theta, double phi, float epsilon, double dt,
            double maxTime) {
        // General Error Format [0] = XERROR [1] = YERROR                
        double[] generalError = Simulate(velocity, hubPosition, 0.1, maxTime, theta);
        double[] EpsilonErrorTheta = Simulate(velocity.changeTheta(epsilon), hubPosition, dt, maxTime, theta);
        double[] EpsilonErrorPhi = Simulate(velocity.changePhi(epsilon), hubPosition, dt, maxTime, theta);
        // DxDy (this is a paratial derivitive)  Format [0] = dx/d("whatever") [1] = dy/d("whatever")
        double[] DxDyTheta = slopeFinder(EpsilonErrorTheta, generalError, epsilon);
        double[] DxDyPhi = slopeFinder(EpsilonErrorPhi, generalError, epsilon);
        double deltaPhi= (DxDyTheta[1]*generalError[0] - DxDyTheta[0]*generalError[1])/(DxDyPhi[1]*DxDyTheta[0]-DxDyPhi[0]*DxDyTheta[1]);
        double deltaTheta = (-generalError[0]-deltaPhi*DxDyPhi[0])/DxDyTheta[0];
        double[] deltaThetaDeltaPhi = {deltaTheta,deltaPhi};
        return deltaThetaDeltaPhi;
    }

    public double[] findThetaPhi(double initalVelocity, Pose3d initalPosition, double initalTheta, float epsilon, int iterationCount,double dt) {
        double initialPhi = initalPosition.getRotation().getZ();
        velocity velocity = new velocity(initalVelocity, initalTheta,initialPhi);
        Transform3d positionDifference = initalPosition.minus(hubPosition);
        double theta = initalTheta;
        double phi  = initialPhi;
        double Diff = Math.sqrt(Math.pow(positionDifference.getX(), 2) + Math.pow(positionDifference.getY(), 2));
        double maxTime = 1 + (Diff / Math.sqrt(Math.pow(velocity.getX(), 2) + Math.pow(velocity.getY(), 2)));
        int iterations = 0;
        while (iterations <= iterationCount){
            double[] deltas = NewtonRappingSon(velocity, theta, phi, epsilon,dt, maxTime);
            velocity = velocity.changePhi(deltas[1]).changeTheta(deltas[0]);
            if (Math.abs((theta + deltas[0])-theta) <= 0.1){
                break;
            }
            theta += deltas[0];
            phi += deltas[1];
        }   
        double[] thetaPhi = {theta,phi};
        return thetaPhi;
    }

    Translation3d crossProduct(Translation3d a, Translation3d b) {
        return new Translation3d(
                a.getY() * b.getZ() - a.getZ() * b.getY(), // x component
                a.getZ() * b.getX() - a.getX() * b.getZ(), // y component
                a.getX() * b.getY() - a.getY() * b.getX() // z component
        );
    }

    private double InchtoMeter(double inch) {
        return inch / 39.37;
    }

    private double[] slopeFinder(double[]EpsilonErrors,double[]Original,float epsilon){
        double dx = (EpsilonErrors[0]-Original[0])/epsilon;
        double dy = (EpsilonErrors[1]-Original[1])/epsilon;
        double[] dxAndDy = {dx,dy};
        return dxAndDy;
    }

    private static class state {
        public Translation3d velocity;
        public Translation3d position;

        public state(Translation3d velocity, Translation3d position) {
            this.velocity = velocity;
            this.position = position;
        }

        public state() {
            this.velocity = new Translation3d();
            this.position = new Translation3d();
        }
    }

    private static class velocity {
        public double velocity;
        public double theta;
        public double phi;

        public velocity(double velocity, double theta, double phi) {
            this.velocity = velocity;
            this.theta = theta;
            this.phi = phi;
        }

        public velocity() {
            this.velocity = 0;
            this.theta = 0;
            this.phi = 0;
        }

        public Double getX() {
            return velocity * Math.cos(theta) * Math.cos(phi);
        }

        public Double getY() {
            return velocity * Math.cos(theta) * Math.sin(phi);
        }

        public Double getZ() {
            return velocity * Math.sin(theta);
        }

        public velocity changeTheta(float epsilon) {
            return new velocity(this.velocity, this.theta + epsilon, this.phi);
        }

        public velocity changePhi(float epsilon) {
            return new velocity(this.velocity, this.theta, this.phi + epsilon);
        }
        public velocity changeTheta(double epsilon) {
            return new velocity(this.velocity, this.theta + epsilon, this.phi);
        }

        public velocity changePhi(double epsilon) {
            return new velocity(this.velocity, this.theta, this.phi + epsilon);
        }
    }
}
