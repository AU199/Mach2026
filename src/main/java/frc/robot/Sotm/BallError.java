package frc.robot.Sotm;

public class BallError {
    private double xError;
    private double yError;

    public BallError(double xError, double yError) {
        this.xError = xError;
        this.yError = yError;
    }

    public double getxError() {
        return xError;
    }

    public double getyError() {
        return yError;
    }
}
