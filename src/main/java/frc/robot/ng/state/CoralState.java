package frc.robot.ng.state;

public enum IntakeState {
    Idle(0),
    Manual(0.5),
    Auto(0.2);

    private IntakeState(double speed) {
        this.speed = speed;
    }

    private double speed;

    public double getSpeed() {
        return this.speed;
    }
}
