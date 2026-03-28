package frc.robot.Sotm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Newton's method optimizer for shoot-on-the-move trajectory.
 *
 * Given a robot pose, velocity, and shooter spin, iteratively finds the
 * elevation angle (theta) and azimuth angle (phi) that land the ball in the hub.
 *
 * Angles are field-relative:
 *   theta = elevation above horizontal (radians)
 *   phi   = azimuth from field +X axis (radians)
 */
public class Newton {

   
    private ChassisSpeeds robotFieldRelativeVelocity;
    private Pose2d robotPose;
    private Pose2d hubPose;
    private double ballAngularSpeedRelativeToRobot; // rad/s, spin speed from shooter wheel
    private double ballInitalSpeedFromShooter;

    
    private static final double DT      = 0.01;  // RK4 timestep (seconds)
    private static final double EPSILON = 1e-3;   // finite difference step for Jacobian

    
    private static final int    MAX_ITER      = 30;
    private static final double CONVERGE_TOL  = 1e-4; // meters — stop when error < 0.1mm

    private Vector finalBallLinearVelocity;
    
    public Newton(
            Pose2d robotPose,
            Pose2d hubPose,
            ChassisSpeeds robotFieldRelativeVelocity,
            double ballInitialSpeedFromShooter,
            double ballInitialSpinFromShooter) {
        this.robotPose = robotPose;
        this.hubPose = hubPose;
        this.robotFieldRelativeVelocity = robotFieldRelativeVelocity;
        this.ballAngularSpeedRelativeToRobot = ballInitialSpinFromShooter;
        this.ballInitalSpeedFromShooter = ballInitialSpeedFromShooter;
    }

    public Vector<N3> calculateBallLinearVelocity(double theta, double phi, double ballSpeed) {
        double heading = robotPose.getRotation().getRadians();

        // ── Ball linear velocity in robot frame ──
        // phi is field-relative, so we first express the shot direction in robot frame
        // by subtracting the robot heading
        double phiRobot = phi - heading;

        double vx_robot = ballSpeed * Math.cos(theta) * Math.cos(phiRobot);
        double vy_robot = ballSpeed * Math.cos(theta) * Math.sin(phiRobot);
        double vz_robot = ballSpeed * Math.sin(theta);

        // ── Rotate ball velocity into field frame and add robot velocity ──
        double vx_field = vx_robot * Math.cos(heading) - vy_robot * Math.sin(heading)
                          + robotFieldRelativeVelocity.vxMetersPerSecond;
        double vy_field = vx_robot * Math.sin(heading) + vy_robot * Math.cos(heading)
                          + robotFieldRelativeVelocity.vyMetersPerSecond;
        double vz_field = vz_robot; // vertical is unchanged by yaw rotation

        Vector<N3> ballLinearVelocity = VecBuilder.fill(vx_field, vy_field, vz_field);

        return ballLinearVelocity;
    }

    public BallError calculateError(double theta, double phi, double speed) {
        Vector<N3> ballLinearVelocity = calculateBallLinearVelocity(theta, phi, speed);
        
        Vector<N3> ballAngularVelocity = VecBuilder.fill(-Math.sin(phi), Math.cos(phi), 0).times(ballAngularSpeedRelativeToRobot);

        // ── Run RK4 ──
        RK4 rk4 = new RK4(
            hubPose,
            robotPose,
            ballLinearVelocity,
            ballAngularVelocity,
            DT
        );

        return rk4.calculateError();
    }

    public Vector getFinalBallLinearVelocity() {
        return finalBallLinearVelocity;
    }

    public ShotAngles findOptimalTrajectory(ShotAngles initialGuess) {
        double theta = initialGuess.getTheta();
        double phi   = initialGuess.getPhi();
        double speed = ballInitalSpeedFromShooter;

        double deltaTheta = 0;
        double deltaPhi = 0;

        double[] iterationThetas = new double[MAX_ITER + 1];
        double[] iterationPhis = new double[MAX_ITER + 1];

        iterationThetas[0] = initialGuess.getTheta();
        iterationPhis[0] = initialGuess.getPhi();

        double[] xErrors = new double[MAX_ITER];
        double[] yErrors = new double[MAX_ITER];

        for (int iter = 0; iter < MAX_ITER; iter++) {
            BallError e0 = calculateError(theta, phi, speed);

            // NaN means trajectory hit ground — give up and return NaN
            if (Double.isNaN(e0.getxError()) || Double.isNaN(e0.getyError())) {
                // Step overshot into bad territory, halve the last step and retry
                // theta -= deltaTheta / 2.0;
                // phi -= deltaPhi / 2.0;
                // continue;

                // Previous comment is complete nonsense, just return NaN and call it a day
                return new ShotAngles(Double.NaN, Double.NaN);
            }

            double ex = e0.getxError();
            double ey = e0.getyError();

            xErrors[iter] = ex;
            yErrors[iter] = ey;

            // Converged
            if (Math.abs(ex) < CONVERGE_TOL && Math.abs(ey) < CONVERGE_TOL) {
                finalBallLinearVelocity = calculateBallLinearVelocity(theta, phi, speed);
                break;
            }

            // ── Numerical Jacobian via forward finite differences ──
            BallError eThetaPlus = calculateError(theta + EPSILON, phi, speed);
            BallError ePhiPlus   = calculateError(theta, phi + EPSILON, speed);

            // Guard: if perturbed trajectories are also NaN, Jacobian is unusable
            if (Double.isNaN(eThetaPlus.getxError()) || Double.isNaN(ePhiPlus.getxError())) {
                finalBallLinearVelocity = calculateBallLinearVelocity(theta, phi, speed);
                break;
            }

            double dEx_dTheta = (eThetaPlus.getxError() - ex) / EPSILON;
            double dEy_dTheta = (eThetaPlus.getyError() - ey) / EPSILON;
            double dEx_dPhi   = (ePhiPlus.getxError()   - ex) / EPSILON;
            double dEy_dPhi   = (ePhiPlus.getyError()   - ey) / EPSILON;

            double det = dEx_dTheta * dEy_dPhi - dEx_dPhi * dEy_dTheta;

            // Singular or near-singular Jacobian — can't invert
            if (Math.abs(det) < 1e-10) {
                finalBallLinearVelocity = calculateBallLinearVelocity(theta, phi, speed);
                break;
            }

            // ── Correct 2x2 Newton step ──
            // Δtheta = -(dEy/dPhi * ex - dEx/dPhi * ey) / det
            // Δphi   = -(-dEy/dTheta * ex + dEx/dTheta * ey) / det
            // deltaTheta = -(dEy_dPhi * ex - dEx_dPhi * ey) / det;
            // deltaPhi   = -(-dEy_dTheta * ex + dEx_dTheta * ey) / det;

            deltaPhi = (dEy_dTheta * ex - dEx_dTheta * ey) / det;
            deltaTheta = (-ey - deltaPhi * dEy_dPhi) / dEy_dTheta;

            theta += deltaTheta;
            phi   += deltaPhi;

            iterationThetas[iter + 1] = theta;
            iterationPhis[iter + 1] = phi;

            // Don't clamp (we just won't shoot if it's over 75)
            // Clamp theta to physically valid range [0, PI/2]
            // theta = Math.max(0.0, Math.min(Math.PI / 2.0, theta));
        }

        // if (theta > Math.toRadians(75)) {
        //     System.out.println("Exceeds radian limit");
        //     return new ShotAngles(Double.NaN, Double.NaN);
        // }

        SmartDashboard.putNumberArray("Iteration Theta", iterationThetas);
        SmartDashboard.putNumberArray("Iteration Phi", iterationPhis);
        SmartDashboard.putNumberArray("X Errors", xErrors);
        SmartDashboard.putNumberArray("Y Errors", yErrors);        

        finalBallLinearVelocity = calculateBallLinearVelocity(theta, phi, speed);
        return new ShotAngles(theta, phi);
    }
}