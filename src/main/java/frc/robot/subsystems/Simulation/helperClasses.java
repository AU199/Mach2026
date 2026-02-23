package frc.robot.subsystems.Simulation;

import edu.wpi.first.math.geometry.Translation3d;

public class helperClasses {
    public helperClasses(){
    }
    public static class state {
        public Translation3d velocity;
        public Translation3d position;

        public state(Translation3d velocity, Translation3d position) {
            this.velocity = velocity;
            this.position = position;
        }

        public state() {
            this.velocity = new Translation3d();
            this.position = new Translation3d();
        }
    }

    public static class velocity {
        public double velocity;
        public double theta;
        public double phi;

        public velocity(double velocity, double theta, double phi) {
            this.velocity = velocity;
            this.theta = theta;
            this.phi = phi;
        }

        public velocity() {
            this.velocity = 0;
            this.theta = 0;
            this.phi = 0;
        }

        public Double getX() {
            return velocity * Math.cos(theta) * Math.cos(phi);
        }

        public Double getY() {
            return velocity * Math.cos(theta) * Math.sin(phi);
        }

        public Double getZ() {
            return velocity * Math.sin(theta);
        }

        public velocity changeTheta(float epsilon) {
            return new velocity(this.velocity, this.theta + epsilon, this.phi);
        }

        public velocity changePhi(float epsilon) {
            return new velocity(this.velocity, this.theta, this.phi + epsilon);
        }

        public velocity changeTheta(double epsilon) {
            return new velocity(this.velocity, this.theta + epsilon, this.phi);
        }

        public velocity changePhi(double epsilon) {
            return new velocity(this.velocity, this.theta, this.phi + epsilon);
        }
        @Override
        public String toString() {
            return String.format(
                    "Velocity[vel = %.3f, ,theta = %.3f, phi = %.3f,x component = %.3f, y component = %.3f, z component = %.3f]",
                    this.velocity, Math.toDegrees(this.theta), Math.toDegrees(this.phi), this.getX(), this.getY(),
                    this.getZ());
        }
    }

}
