package frc.robot.Sotm;

import edu.wpi.first.math.Vector;

public class BallState {
    private Vector position;
    private Vector velocity;

    public BallState(Vector position, Vector velocity) {
        this.position = position;
        this.velocity = velocity;
    }

    public Vector getPosition() {
        return position;
    }

    public Vector getVelocity() {
        return velocity;
    }
}
