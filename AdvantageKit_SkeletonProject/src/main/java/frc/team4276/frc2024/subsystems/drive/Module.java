package frc.team4276.frc2024.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.littletonrobotics.junction.Logger;

public class Module {
    private final int index;
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private SwerveModuleState setpointState = new SwerveModuleState();

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
    }

    /** Called while blocking odometry thread */
    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + index, inputs);
    }

    /** Runs to {@link SwerveModuleState} */
    public void runSetpoint(SwerveModuleState setpoint) {
        io.runDriveVelocitySetpoint(
                setpoint.speedMetersPerSecond, 0.0);
        io.runTurnPositionSetpoint(setpoint.angle.getRadians());
    }

    public SwerveModuleState getSetpointState() {
        return setpointState;
    }

    /**
     * Runs characterization volts or amps depending on using voltage or current
     * control.
     */
    public void runCharacterization(double turnSetpointRads, double input) {
        io.runTurnPositionSetpoint(turnSetpointRads);
        io.runCharacterization(input);
    }

    /** Sets brake mode to {@code enabled}. */
    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }

    /** Stops motors. */
    public void stop() {
        io.stop();
    }

    /** Get all latest {@link SwerveModulePosition}'s from last cycle. */
    public SwerveModulePosition[] getModulePositions() {
        int minOdometryPositions = Math.min(inputs.odometryDrivePositionsMeters.length,
                inputs.odometryTurnPositions.length);
        SwerveModulePosition[] positions = new SwerveModulePosition[minOdometryPositions];
        for (int i = 0; i < minOdometryPositions; i++) {
            positions[i] = new SwerveModulePosition(
                    inputs.odometryDrivePositionsMeters[i], inputs.odometryTurnPositions[i]);
        }
        return positions;
    }

    /** Get turn angle of module in {@link Rotation2d} */
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(inputs.turnPositionRads);
    }

    /** Get turn angle of module in radians */
    public double getAngleRads() {
        return inputs.turnPositionRads;
    }

    /** Get position of wheel rotations in radians */
    public double getPositionMetres() {
        return inputs.drivePositionMetres;
    }

    /** Get velocity of wheel in m/s. */
    public double getVelocityMetresPerSec() {
        return inputs.driveVelocityMetresPerSec;
    }

    /** Get current {@link SwerveModulePosition} of module. */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMetres(), getAngle());
    }

    /** Get current {@link SwerveModuleState} of module. */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetresPerSec(), getAngle());
    }

    /** Get velocity of drive wheel for characterization */
    public double getCharacterizationVelocity() {
        return inputs.driveVelocityMetresPerSec;
    }
}
