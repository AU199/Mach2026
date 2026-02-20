package frc.robot.Sotm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

public class Newton {
    private ChassisSpeeds robotRobotRelativeVelocity;
    private ChassisSpeeds robotFieldRelativeVelocity;
    
    private Vector ballInitialLinearVelocityRelativeToRobot;
    private Vector ballInitialAngularVelocityRelativeToRobot;
    private Vector ballInitialLinearVelocityRelativeToField;
    private Vector ballInitialAngularVelocityRelativeToField;

    private Pose2d shooterPose;
    private Pose2d robotPose;
    private Pose2d hubPose;
    private Pose2d targetPose;

    private double dt = 0.05;
    private double epsilon = 0.001;

    public Newton(Pose2d shooterPose, Pose2d robotPose, Pose2d hubPose, ChassisSpeeds robotVelocity) {
        this.shooterPose = shooterPose;
        this.robotPose = robotPose;
        this.robotRobotRelativeVelocity = robotVelocity;
        this.robotFieldRelativeVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, robotPose.getRotation());
        this.hubPose = hubPose;
    }

    public BallError calculateError(double theta, double phi, double ballInitialLinearSpeedRelativeToShooter, double ballInitialAngularSpeedRelativeToShooter) {
        double heading = robotPose.getRotation().getRadians();
        double wx_robot = ballInitialAngularSpeedRelativeToShooter * Math.cos(phi);
        double wy_robot = ballInitialAngularSpeedRelativeToShooter * Math.sin(phi);
        
        ballInitialLinearVelocityRelativeToRobot = VecBuilder.fill(
            ballInitialLinearSpeedRelativeToShooter * Math.cos(theta) * Math.cos(phi), 
            ballInitialLinearSpeedRelativeToShooter * Math.cos(theta) * Math.sin(phi),
            ballInitialLinearSpeedRelativeToShooter * Math.sin(theta));

        double vx_robot = ballInitialLinearVelocityRelativeToRobot.get(0);
        double vy_robot = ballInitialLinearVelocityRelativeToRobot.get(1);
        double vz_robot = ballInitialLinearVelocityRelativeToRobot.get(2);

        ballInitialAngularVelocityRelativeToRobot = VecBuilder.fill( 0, ballInitialAngularSpeedRelativeToShooter, 0);

        double vx_field = vx_robot * Math.cos(heading) - vy_robot * Math.sin(heading);
        double vy_field = vx_robot * Math.sin(heading) + vy_robot * Math.cos(heading);
        ballInitialLinearVelocityRelativeToField = VecBuilder.fill(
            vx_field + robotFieldRelativeVelocity.vxMetersPerSecond,
            vy_field + robotFieldRelativeVelocity.vyMetersPerSecond,
            vz_robot);

        ballInitialAngularVelocityRelativeToField = VecBuilder.fill(
            wx_robot * Math.cos(heading) - wy_robot * Math.sin(heading),
            wx_robot * Math.sin(heading) + wy_robot * Math.cos(heading),
            0); // Claude code please check

        targetPose = hubPose; // Add ability to change this later
        RK4 rk4 = new RK4(targetPose, shooterPose, robotFieldRelativeVelocity, ballInitialLinearVelocityRelativeToField, ballInitialAngularVelocityRelativeToField, shooterPose, dt);

        return rk4.calculateError();
    }

    public ShotAngles findOptimalTrajectory(ShotAngles currentGuess) {
        double angularVelocity = Constants.ballInitialAngularVelocity;
        // double angularVelocity = Math.sqrt(Math.pow(robotFieldRelativeVelocity.vxMetersPerSecond, 2) + Math.pow(robotFieldRelativeVelocity.vyMetersPerSecond, 2))/Constants.robotRadius;
        BallError idealErrors, thetaAdjustedErrors, phiAdjustedErrors;

        double ballInitialVelocity = Constants.ballInitialVelocity; // Find launch speed later
        idealErrors = calculateError(currentGuess.getTheta(), currentGuess.getPhi(), ballInitialVelocity, angularVelocity);
        thetaAdjustedErrors = calculateError(currentGuess.getTheta() + epsilon, currentGuess.getPhi(), ballInitialVelocity, angularVelocity);
        phiAdjustedErrors = calculateError(currentGuess.getTheta(), currentGuess.getPhi() + epsilon, ballInitialVelocity, angularVelocity);

        Derivative thetaDerivatives = new Derivative(thetaAdjustedErrors.getxError() - idealErrors.getxError(), thetaAdjustedErrors.getyError() - idealErrors.getyError(), epsilon);
        Derivative phiDerivatives = new Derivative(phiAdjustedErrors.getxError() - idealErrors.getxError(), phiAdjustedErrors.getyError() - idealErrors.getyError(), epsilon);

        // Jacobians are from Claude please check
        double dEx_dTheta = thetaDerivatives.getDerivatives()[0];
        double dEy_dTheta = thetaDerivatives.getDerivatives()[1];
        double dEx_dPhi = phiDerivatives.getDerivatives()[0];
        double dEy_dPhi = phiDerivatives.getDerivatives()[1];

        double ex = idealErrors.getxError();
        double ey = idealErrors.getyError();
        double det = dEx_dTheta * dEy_dPhi - dEy_dTheta * dEx_dPhi;

        double deltaTheta = (-ex * dEy_dPhi + ey * dEx_dPhi) / det;
        double deltaPhi = (-ey * dEx_dTheta + ex * dEy_dTheta) / det;

        return new ShotAngles(currentGuess.getTheta() + deltaTheta, currentGuess.getPhi() + deltaPhi);
    }
}