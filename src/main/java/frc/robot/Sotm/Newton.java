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
    private double ballInitialAngularSpeedRelativeToRobot;
    private Vector ballInitialLinearVelocityRelativeToField;
    private Vector ballInitialAngularVelocityRelativeToField;

    private Pose2d shooterPose;
    private Pose2d robotPose;
    private Pose2d hubPose;
    private Pose2d targetPose;

    private double dt = 0.001;
    private double epsilon = 1e-4;

    public Newton(Pose2d shooterPose, Pose2d robotPose, Pose2d hubPose, ChassisSpeeds robotVelocity, double angularSpeedRelativeToRobot) {
        this.shooterPose = shooterPose;
        this.robotPose = robotPose;
        this.robotRobotRelativeVelocity = robotVelocity;
        this.robotFieldRelativeVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, robotPose.getRotation());
        this.hubPose = hubPose;
        this.ballInitialAngularSpeedRelativeToRobot = angularSpeedRelativeToRobot;
    }

    public BallError calculateError(double theta, double phi, double ballInitialLinearSpeedRelativeToShooter) {
        double heading = robotPose.getRotation().getRadians();
        double wx_robot = -ballInitialAngularSpeedRelativeToRobot * Math.cos(phi);
        double wy_robot = ballInitialAngularSpeedRelativeToRobot * Math.sin(phi);
        
        ballInitialLinearVelocityRelativeToRobot = VecBuilder.fill(
            ballInitialLinearSpeedRelativeToShooter * Math.cos(theta) * Math.cos(phi), 
            ballInitialLinearSpeedRelativeToShooter * Math.cos(theta) * Math.sin(phi),
            ballInitialLinearSpeedRelativeToShooter * Math.sin(theta));

        double vx_robot = ballInitialLinearVelocityRelativeToRobot.get(0);
        double vy_robot = ballInitialLinearVelocityRelativeToRobot.get(1);
        double vz_robot = ballInitialLinearVelocityRelativeToRobot.get(2);

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
        // double angularVelocity = Math.sqrt(Math.pow(robotFieldRelativeVelocity.vxMetersPerSecond, 2) + Math.pow(robotFieldRelativeVelocity.vyMetersPerSecond, 2))/Constants.robotRadius;
        BallError idealErrors, thetaAdjustedErrors, phiAdjustedErrors;

        double ballInitialVelocity = Constants.ballInitialVelocityFromShooter; // Find launch speed later
        idealErrors = calculateError(currentGuess.getTheta(), currentGuess.getPhi(), ballInitialVelocity);
        thetaAdjustedErrors = calculateError(currentGuess.getTheta() + epsilon, currentGuess.getPhi(), ballInitialVelocity);
        phiAdjustedErrors = calculateError(currentGuess.getTheta(), currentGuess.getPhi() + epsilon, ballInitialVelocity);

        Derivative thetaDerivatives = new Derivative(thetaAdjustedErrors.getxError() - idealErrors.getxError(), thetaAdjustedErrors.getyError() - idealErrors.getyError(), epsilon);
        Derivative phiDerivatives = new Derivative(phiAdjustedErrors.getxError() - idealErrors.getxError(), phiAdjustedErrors.getyError() - idealErrors.getyError(), epsilon);

        // Jacobians are from Claude please check
        double dEx_dTheta = thetaDerivatives.getDerivatives()[0];
        double dEy_dTheta = thetaDerivatives.getDerivatives()[1];
        double dEx_dPhi = phiDerivatives.getDerivatives()[0];
        double dEy_dPhi = phiDerivatives.getDerivatives()[1];

        double ex = idealErrors.getxError();
        double ey = idealErrors.getyError();
        double det = dEx_dTheta * dEy_dPhi - dEx_dPhi * dEy_dTheta;

        double deltaPhi = (ex * dEy_dTheta - ey * dEx_dTheta) / det;
        double deltaTheta = (-ex - dEy_dPhi * dEx_dPhi) / dEx_dTheta;

        return new ShotAngles(currentGuess.getTheta() + deltaTheta, currentGuess.getPhi() + deltaPhi);
    }
}