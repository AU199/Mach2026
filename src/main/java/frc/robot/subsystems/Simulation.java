package frc.robot.subsystems;

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
import frc.robot.FuelSim.Hub;
import org.littletonrobotics.junction.Logger;

public class Simulation extends SubsystemBase {
    // Mass of the ball
    private double mass = 0.203;
    private Translation3d gravity = new Translation3d(0, 0, -9.81);
    private double CD;
    private double CL;
    // Air density kg/m^3
    private double rho = 1.2;
    // radius of the ball
    private double radius = 0.075;
    private double hubRadius = InchtoMeter(41.73);
    private Translation3d omega;
    private Pose3d hubPosition;
    private int iterations = 0;
    StructPublisher<Pose3d> hubPositionPublisherSIMULATION = NetworkTableInstance.getDefault()
            .getStructTopic("Hub PositionSIMULATION", Pose3d.struct).publish();

    double A = Math.PI * Math.pow(radius, 2);

    public Simulation(double CD, double CL, Translation3d Omega, Pose3d hubPosition) {
        this.CD = CD;
        this.CL = CL;
        this.omega = Omega;
        this.hubPosition = hubPosition;
    }

    /**
     * @param currVelocity the current velocity of the ball
     * @return The acceleration vector
     */
    public Translation3d acceleration(Translation3d currVelocity) {
        // if (currVelocity.getNorm() < 1e-5) {
        // return gravity;
        // }
        double DragMag = (0.5 * rho * CD * A * Math.pow(currVelocity.getNorm(), 2)) / mass;
        Translation3d DragAccel = currVelocity.times(-DragMag / currVelocity.getNorm());
        double magnusCoeff = (0.5 * rho * CL * A * radius) / mass;
        Translation3d MagnusAccel = crossProduct(omega, currVelocity).times(magnusCoeff);
        return gravity.plus(DragAccel).plus(MagnusAccel);
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
    private state rk4Step(Translation3d velocity, Translation3d position, double dt) {
        Translation3d tempVel = velocity;
        Translation3d tempPos = position;

        Translation3d k1_v = acceleration(tempVel);
        Translation3d k1_x = tempVel;

        Translation3d v2 = tempVel.plus(k1_v.times(dt / 2));
        Translation3d x2 = tempPos.plus(k1_x.times(dt / 2));
        Translation3d k2_v = acceleration(v2);
        Translation3d k2_x = v2;

        Translation3d v3 = tempVel.plus(k2_v.times(dt / 2));
        Translation3d x3 = tempPos.plus(k2_x.times(dt / 2));
        Translation3d k3_v = acceleration(v3);
        Translation3d k3_x = v3;

        Translation3d v4 = tempVel.plus(k3_v.times(dt));
        Translation3d x4 = tempPos.plus(k3_x.times(dt));
        Translation3d k4_v = acceleration(v4);
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
    public double[] Simulate(velocity initalVelocity, Pose3d initalPosition, double dt, double maxTime,
            Pose3d hubPosition) {
        Translation3d pos = new Translation3d(initalPosition.getX(), initalPosition.getY(), initalPosition.getZ());
        Translation3d vel = new Translation3d(initalVelocity.getX(), initalVelocity.getY(), initalVelocity.getZ());
        // System.out.println(vel.getZ());
        double time = 0;
        while (true) {
            double horizontalDistance = pos.toTranslation2d().getDistance(hubPosition.toPose2d().getTranslation());
            // System.out.println("Horizontal distance "+ horizontalDistance);
            if (horizontalDistance < hubRadius && pos.getZ() <= hubPosition.getZ()) {
                double zError = hubPosition.getZ() - pos.getZ();
                // System.out.println("Z error" + hubPosition.getZ()+ " "+ pos.getZ());
                // System.out.println("X pos" + pos.getX());
                // System.out.println("Y pos" + pos.getY());

                break;
            }

            if (time > maxTime) {
                break;
            }

            state rk4State = rk4Step(vel, pos, dt);
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
     * Uses the Newton-Rapson method in order to calculate the changes in theta and
     * phi in order to improve shot accuracy.
     * 
     * @param velocity    The velocity of the fuel
     * @param Position    the position of the fuel with respect to the field origin
     * @param epsilon     the epsilon used to calculate the derivitive of the theta
     *                    and phi function
     * @param dt          the small step in time for the simulation
     * @param maxTime     the maximum simulation time (NOT THE CPU TIME)
     * @param hubPosition the position of the hub with respect to the field origin
     * @return the change of theta and phi
     */
    public double[] NewtonRappingSon(velocity velocity, Pose3d Position, Float epsilon, double dt,
            double maxTime, Pose3d hubPosition) {

        // General Error Format [0] = XERROR [1] = YERROR
        double[] generalError = Simulate(velocity, Position, dt, maxTime, hubPosition);
        // System.out.println(generalError[0] + " " + generalError[1]);
        double[] EpsilonErrorTheta = Simulate(velocity.changeTheta(epsilon), Position, dt, maxTime, hubPosition);
        // System.out.println("EX" + EpsilonErrorTheta[0] + " " + generalError[0]);
        double[] EpsilonErrorPhi = Simulate(velocity.changePhi(epsilon), Position, dt, maxTime, hubPosition);
        // System.out.println("E" + EpsilonErrorPhi[0] + " " + EpsilonErrorPhi[1]);

        // DxDy (this is a paratial derivitive) Format [0] = dx/d("whatever") [1] =
        // dy/d("whatever")
        double[] DxDyTheta = slopeFinder(EpsilonErrorTheta, generalError, epsilon);
        // SmartDashboard.putNumberArray("paritial derivitve theta ", DxDyTheta);
        // System.out.println(DxDyTheta[0]+" "+DxDyTheta[1]);
        double[] DxDyPhi = slopeFinder(EpsilonErrorPhi, generalError, epsilon);
        // System.out.println(DxDyPhi[0] +' '+DxDyPhi[1]);
        // DeltaPhi = c*xerror - a*yerror/(ad-bc)
        double det = DxDyPhi[1] * DxDyTheta[0] - DxDyPhi[0] * DxDyTheta[1];
        if (Math.abs(det) < 1e-10) {
            System.out.println(det);
            return new double[] { 0, 0, generalError[0], generalError[1] };
        }
        double deltaPhi = (DxDyTheta[1] * generalError[0] - DxDyTheta[0] * generalError[1]) / det;
        // DeltaTheta = (-xerror - deltaPhi*b)/a
        double deltaTheta = (-generalError[0] - deltaPhi * DxDyPhi[0]) / DxDyTheta[0];

        return new double[] { deltaTheta, deltaPhi, generalError[0], generalError[1] };
    }

    /**
     * @param initalVelocity The inital velocity of the fuel
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
    public double[] findThetaPhi(double initalVelocity, Pose3d initalPosition,
            double initalTheta, Float epsilon,
            int iterationCount, double dt, double tolerance) {
        // System.out.println(hubPosition);
        double initialPhi = initalPosition.getRotation().getZ();
        velocity velocity = new velocity(initalVelocity, initalTheta, initialPhi);
        System.out.println(velocity);
        hubPosition = this.hubPosition;
        Transform3d positionDifference = initalPosition.minus(hubPosition);
        System.out.println(hubPosition);
        double theta = initalTheta;
        double phi = initialPhi;
        double Diff = Math.sqrt(Math.pow(positionDifference.getX(), 2) + Math.pow(positionDifference.getY(), 2));
        this.iterations = 0;
        while (iterations <= iterationCount) {
            double horizontalSpeed = Math.sqrt(Math.pow(velocity.getX(), 2) + Math.pow(velocity.getY(), 2));
            double maxTime = (Diff / Math.max(horizontalSpeed, 0.1));

            double[] deltas = NewtonRappingSon(velocity, initalPosition, epsilon, dt, maxTime, hubPosition);
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

    private double InchtoMeter(double inch) {
        return inch / 39.37;
    }

    /**
     * @param EpsilonErrors The x and y errors of the added epsilon
     * @param Original The x any y errors of the original shot
     * @param epsilon the epsilon (a tiny step)
     * @return double[]
     */
    private double[] slopeFinder(double[] EpsilonErrors, double[] Original, float epsilon) {
        double dx = (EpsilonErrors[0] - Original[0]) / epsilon;
        double dy = (EpsilonErrors[1] - Original[1]) / epsilon;
        double[] dxAndDy = { dx, dy };
        return dxAndDy;
    }

    private class state {
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

    private class velocity {
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
        @Override
        public String toString() {
            return String.format(
                    "Velocity[vel = %.3f, ,theta = %.3f, phi = %.3f,x component = %.3f, y component = %.3f, z component = %.3f]",
                    this.velocity, Math.toDegrees(this.theta), Math.toDegrees(this.phi), this.getX(), this.getY(),
                    this.getZ());
        }
    }

    @Override
    public void periodic() {
        hubPositionPublisherSIMULATION
                .accept(new Pose3d(hubPosition.getX(), hubPosition.getY(), 72 / 39.37, new Rotation3d(0, 0, 0)));
    }
}
