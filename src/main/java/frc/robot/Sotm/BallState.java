package frc.robot.Sotm;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class BallState {

    private Vector<N3> position;
    private Vector<N3> velocity;

    public BallState(Vector<N3> position, Vector<N3> velocity) {
        this.position = position;
        this.velocity = velocity;
    }

    public Vector<N3> getPosition() {
        return position;
    }

    public Vector<N3> getVelocity() {
        return velocity;
    }
}
