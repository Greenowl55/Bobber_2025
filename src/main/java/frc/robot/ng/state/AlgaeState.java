package frc.robot.ng.state;

public enum AlgaeState {
    Idle(0),
    RollIn(0.2),
    RollOut(-0.2);

    private AlgaeState(double speed) {
        this.speed = speed;
    }

    private double speed;

    public double getSpeed() {
        return this.speed;
    }
}
