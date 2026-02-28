package frc.robot.Sotm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.FuelSim;

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

   
    private final ChassisSpeeds robotFieldRelativeVelocity;
    private final Pose2d robotPose;
    private final Pose2d hubPose;
    private final double ballAngularSpeedRelativeToRobot; // rad/s, spin speed from shooter wheel

    
    private static final double DT      = 0.001;  // RK4 timestep (seconds)
    private static final double EPSILON = 1e-4;   // finite difference step for Jacobian

    
    private static final int    MAX_ITER      = 30;
    private static final double CONVERGE_TOL  = 1e-4; // meters — stop when error < 0.1mm

    

    
    public Newton(
            Pose2d robotPose,
            Pose2d hubPose,
            ChassisSpeeds robotFieldRelativeVelocity) {
        this.robotPose = robotPose;
        this.hubPose = hubPose;
        this.robotFieldRelativeVelocity = robotFieldRelativeVelocity;
        this.ballAngularSpeedRelativeToRobot = Constants.ballInitialSpinFromShooter;
    }

    
    public BallError calculateError(double theta, double phi, double speed) {
        double heading = robotPose.getRotation().getRadians();

        // ── Ball linear velocity in robot frame ──
        // phi is field-relative, so we first express the shot direction in robot frame
        // by subtracting the robot heading
        double phiRobot = phi - heading;

        double vx_robot = speed * Math.cos(theta) * Math.cos(phiRobot);
        double vy_robot = speed * Math.cos(theta) * Math.sin(phiRobot);
        double vz_robot = speed * Math.sin(theta);

        // ── Rotate ball velocity into field frame and add robot velocity ──
        double vx_field = vx_robot * Math.cos(heading) - vy_robot * Math.sin(heading)
                          + robotFieldRelativeVelocity.vxMetersPerSecond;
        double vy_field = vx_robot * Math.sin(heading) + vy_robot * Math.cos(heading)
                          + robotFieldRelativeVelocity.vyMetersPerSecond;
        double vz_field = vz_robot; // vertical is unchanged by yaw rotation

        Vector<N3> ballLinearVelocity = VecBuilder.fill(vx_field, vy_field, vz_field);

        // ── Ball angular velocity ──
        // Shooter wheel spins around an axis perpendicular to the shot direction
        // (pure backspin/topspin). Spin axis in robot frame is perpendicular to shot
        // direction in the horizontal plane: (-sin(phiRobot), cos(phiRobot), 0)
        // This produces backspin (top of ball moving opposite to travel).
        double wx_robot = -ballAngularSpeedRelativeToRobot * Math.sin(phiRobot);
        double wy_robot =  ballAngularSpeedRelativeToRobot * Math.cos(phiRobot);

        // Rotate angular velocity into field frame
        double wx_field = wx_robot * Math.cos(heading) - wy_robot * Math.sin(heading);
        double wy_field = wx_robot * Math.sin(heading) + wy_robot * Math.cos(heading);

        Vector<N3> ballAngularVelocity = VecBuilder.fill(wx_field, wy_field, 0.0);

        // ── Run RK4 ──
        RK4 rk4 = new RK4(
            hubPose,
            robotPose,
            ballLinearVelocity,
            ballAngularVelocity,
            DT
        );

        return rk4.calculateError(ballLinearVelocity);
    }

    public ShotAngles findOptimalTrajectory(ShotAngles initialGuess) {
        double theta = initialGuess.getTheta();
        double phi   = initialGuess.getPhi();
        double speed = Constants.ballInitialVelocityFromShooter;

        for (int iter = 0; iter < MAX_ITER; iter++) {
            BallError e0 = calculateError(theta, phi, speed);

            // NaN means trajectory hit ground — give up and return current best
            if (Double.isNaN(e0.getxError()) || Double.isNaN(e0.getyError())) {
                break;
            }

            double ex = e0.getxError();
            double ey = e0.getyError();

            // Converged
            if (Math.abs(ex) < CONVERGE_TOL && Math.abs(ey) < CONVERGE_TOL) {
                break;
            }

            // ── Numerical Jacobian via forward finite differences ──
            BallError eThetaPlus = calculateError(theta + EPSILON, phi, speed);
            BallError ePhiPlus   = calculateError(theta, phi + EPSILON, speed);

            // Guard: if perturbed trajectories are also NaN, Jacobian is unusable
            if (Double.isNaN(eThetaPlus.getxError()) || Double.isNaN(ePhiPlus.getxError())) {
                break;
            }

            double dEx_dTheta = (eThetaPlus.getxError() - ex) / EPSILON;
            double dEy_dTheta = (eThetaPlus.getyError() - ey) / EPSILON;
            double dEx_dPhi   = (ePhiPlus.getxError()   - ex) / EPSILON;
            double dEy_dPhi   = (ePhiPlus.getyError()   - ey) / EPSILON;

            double det = dEx_dTheta * dEy_dPhi - dEx_dPhi * dEy_dTheta;

            // Singular or near-singular Jacobian — can't invert
            if (Math.abs(det) < 1e-10) {
                break;
            }

            // ── Correct 2x2 Newton step ──
            // Δtheta = -(dEy/dPhi * ex - dEx/dPhi * ey) / det
            // Δphi   = -(-dEy/dTheta * ex + dEx/dTheta * ey) / det
            double deltaTheta = -(dEy_dPhi * ex - dEx_dPhi * ey) / det;
            double deltaPhi   = -(-dEy_dTheta * ex + dEx_dTheta * ey) / det;

            theta += deltaTheta;
            phi   += deltaPhi;

            // Don't clamp (we just won't shoot if it's over 75)
            // Clamp theta to physically valid range [0, PI/2]
            // theta = Math.max(0.0, Math.min(Math.PI / 2.0, theta));
        }

        if (theta > 75) {
            return new ShotAngles(Double.NaN, Double.NaN);
        }
        return new ShotAngles(theta, phi);
    }
}
