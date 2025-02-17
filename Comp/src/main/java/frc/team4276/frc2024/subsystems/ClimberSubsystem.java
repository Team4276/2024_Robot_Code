package frc.team4276.frc2024.subsystems;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;
import frc.team4276.frc2024.Constants.ClimberSubsystemConstants;
import frc.team4276.lib.drivers.ServoMotorSubsystem;

public class ClimberSubsystem extends ServoMotorSubsystem {
    public enum VoltageState {
        IDLE(0.0),
        RAISE(0.0),
        LOWER(0.0);

        public double voltage;

        private VoltageState(double voltage) {
            this.voltage = voltage;
        }
    }

    public enum SetpointState {
        STOW(0.0),
        LOW_RAISE(0.0),
        HIGH_RAISE(0.0);

        public double position;

        private SetpointState(double position) {
            this.position = position;
        }
    }

    private static ClimberSubsystem mInstance;

    public static ClimberSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ClimberSubsystem();
        }

        return mInstance;
    }

    private ClimberSubsystem() {
        super(ClimberSubsystemConstants.kClimberServoConstants);

        burnFlash();
    }

    public void setSetpointState(SetpointState state) {
        setFuseMotionSetpoint(state.position);
    }

    public void setVoltageState(VoltageState state) {
        setVoltage(state.voltage);
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                setVoltage(0.0);
                setWantBrakeMode(true);
            }

            @Override
            public void onLoop(double timestamp) {
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }
}
