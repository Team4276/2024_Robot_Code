package frc.team4276.frc2024.subsystems.feedtake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

public class Roller extends GenericRollerSystem<Roller.VoltageGoal> {

    public enum Goal implements GenericRollerSystem.VoltageGoal {
        IDLE(() -> 0.0),
        INTAKE(() -> 12.0),
        SLOW_FEED(() -> 4.0),
        AMP(() -> 10.0),
        DEFEED(() -> -2.0),
        EXHAUST(() -> -8.0),
        SHOOT(() -> 12.0);

        private final DoubleSupplier voltageSupplier;

        private Goal(DoubleSupplier voltageSupplier) {
            this.voltageSupplier = voltageSupplier;
        }

        @Override
        public DoubleSupplier getVoltageSupplier() {
            return voltageSupplier;
        }
    }

    private Goal goal = Goal.IDLE;

    public Roller(GenericRollerSystemIO io){
        super("Roller", io);
    }

    @Override
    public VoltageGoal getGoal() {
        return goal;
    }

    @AutoLogOutput
    public void setGoal(Goal goal) {
        this.goal = goal;
    }
}
