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
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.Simulation.helperClasses.*;
import frc.robot.util.FuelSim.Hub;
import frc.robot.subsystems.Simulation.RK4;

public class NewtonRapson extends SubsystemBase {
    // Mass of the ball
    private double CD;
    private double CL;
    public static RK4 rk4Simulator;
    // Air density kg/m^3
    // radius of the ball
    private double hubRadius = InchtoMeter(41.73);
    private Pose3d hubPosition;
    private int iterations = 0;
    StructPublisher<Pose3d> hubPositionPublisherSIMULATION = NetworkTableInstance.getDefault()
            .getStructTopic("Hub PositionSIMULATION", Pose3d.struct).publish();


    public NewtonRapson(double CD, double CL, Pose3d hubPosition) {
        this.CD = CD;
        this.CL = CL;
        this.hubPosition = hubPosition;
        NewtonRapson.rk4Simulator = new RK4(CD, CL, hubRadius);

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
    public double[] NewtonRappingSon(velocity velocity, Translation3d omega, Pose3d Position, Float epsilon, double dt,
            double maxTime, Pose3d hubPosition) {

        // General Error Format [0] = XERROR [1] = YERROR
        double[] generalError = rk4Simulator.getXandYError(velocity, omega, Position, dt, maxTime, hubPosition);
        // System.out.println(generalError[0] + " " + generalError[1]);
        double[] EpsilonErrorTheta = rk4Simulator.getXandYError(velocity.changeTheta(epsilon), omega, Position, dt, maxTime, hubPosition);
        // System.out.println("EX" + EpsilonErrorTheta[0] + " " + generalError[0]);
        double[] EpsilonErrorPhi = rk4Simulator.getXandYError(velocity.changePhi(epsilon), omega, Position, dt, maxTime, hubPosition);
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

    
}
