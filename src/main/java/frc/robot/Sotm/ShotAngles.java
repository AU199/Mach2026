package frc.robot.Sotm;

public class ShotAngles {
    private double theta;
    private double phi;
    
    public ShotAngles(double theta, double phi) {
        this.theta = theta;
        this.phi = phi;
    }

    public double getTheta() {
        return theta;
    }

    public double getPhi() {
        return phi;
    }
}
