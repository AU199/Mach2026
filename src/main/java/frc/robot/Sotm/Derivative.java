package frc.robot.Sotm;

public class Derivative {
    private double dx;
    private double dy;
    private double dtheta;
    private double dphi;

    public Derivative(double xError, double yError, double epsilon) {
        this.dx = xError/epsilon;
        this.dy = yError/epsilon;
    }

    public double[] getDerivatives(){
        return new double[]{dx, dy};
    }
}
