package frc.robot.Sotm;

public class ShotAngles {
    private double theta; // elevation angle (radians, 0 = horizontal, PI/2 = straight up)
    private double phi;   // azimuth angle (radians, field-relative)

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
