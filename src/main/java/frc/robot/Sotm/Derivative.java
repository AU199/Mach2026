package frc.robot.Sotm;

/**
 * Stores partial derivatives of (xError, yError) with respect to one angle parameter.
 * dx = d(xError)/d(angle), dy = d(yError)/d(angle)
 */
public class Derivative {
    private double dx;
    private double dy;

    public Derivative(double xError, double yError, double epsilon) {
        this.dx = xError / epsilon;
        this.dy = yError / epsilon;
    }

    public double[] getDerivatives() {
        return new double[]{dx, dy};
    }
}
