package frc.robot.commands;

public class FishHookState {
    public enum State{

        IDLE(0.0),
        L1(35.0),
        L2(35.0),
        L3(35.0),
        L4(50.0);

        private double angleDeg;

        State(double angleDeg){
            this.angleDeg = angleDeg;
        }

        public double getAngleDeg(){
            return angleDeg;
        }
    }
}