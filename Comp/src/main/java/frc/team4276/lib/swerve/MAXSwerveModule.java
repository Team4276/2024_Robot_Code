// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.lib.swerve;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Constants.MaxSwerveModuleConstants;
import frc.team4276.lib.drivers.Subsystem;
import frc.team4276.lib.rev.CANSparkMaxFactory;
import frc.team4276.lib.rev.VIKCANSparkMax;

import frc.team1678.lib.swerve.ModuleState;

import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.util.Util;

public class MAXSwerveModule extends Subsystem {
    private final VIKCANSparkMax mDrive;
    private final VIKCANSparkMax mTurn;

    private final RelativeEncoder mDriveEncoder;
    private final AbsoluteEncoder mTurnEncoder;

    private final PeriodicIO mPeriodicIO;

    private final MAXSwerveModuleConstants mConstants;

    public static class MAXSwerveModuleConstants {
        public String kName = "ERROR_ASSIGN_A_NAME";
        public int kDriveId = -1;
        public int kTurnId = -1;
        public double kOffset = 0.0;
    }

    public MAXSwerveModule(MAXSwerveModuleConstants constants) {
        mConstants = constants;
        mDrive = CANSparkMaxFactory.createDefault(mConstants.kDriveId);
        mTurn = CANSparkMaxFactory.createDefault(mConstants.kTurnId);

        mDriveEncoder = mDrive.getEncoder();
        mTurnEncoder = CANSparkMaxFactory.configAbsoluteEncoder(mTurn, MaxSwerveModuleConstants.kTurningEncoderConfig);
        mDrive.getPIDController().setFeedbackDevice(mDriveEncoder);
        mTurn.getPIDController().setFeedbackDevice(mTurnEncoder);

        mDriveEncoder.setPositionConversionFactor(MaxSwerveModuleConstants.kDrivingEncoderPositionFactor);
        mDriveEncoder.setVelocityConversionFactor(MaxSwerveModuleConstants.kDrivingEncoderVelocityFactor);

        mTurn.getPIDController().setPositionPIDWrappingEnabled(true);
        mTurn.getPIDController()
                .setPositionPIDWrappingMinInput(MaxSwerveModuleConstants.kTurningEncoderPositionPIDMinInput);
        mTurn.getPIDController()
                .setPositionPIDWrappingMaxInput(MaxSwerveModuleConstants.kTurningEncoderPositionPIDMaxInput);

        CANSparkMaxFactory.configPIDF(mDrive, MaxSwerveModuleConstants.kDrivingPIDFConfig);
        CANSparkMaxFactory.configPIDF(mTurn, MaxSwerveModuleConstants.kTurningPIDFConfig);

        mDrive.setIdleMode(MaxSwerveModuleConstants.kDrivingMotorIdleMode);
        mTurn.setIdleMode(MaxSwerveModuleConstants.kTurningMotorIdleMode);
        mDrive.setSmartCurrentLimit(MaxSwerveModuleConstants.kDrivingMotorCurrentLimit);
        mTurn.setSmartCurrentLimit(MaxSwerveModuleConstants.kTurningMotorCurrentLimit);

        mDriveEncoder.setPosition(0);

        mDrive.burnFlash();
        mTurn.burnFlash();

        mPeriodicIO = new PeriodicIO();

        Shuffleboard.getTab("Swerve").addDouble(mConstants.kName + " Rotation Demand", () -> mPeriodicIO.rotationDemand);
        Shuffleboard.getTab("Swerve").addDouble(mConstants.kName + " Turn Position", () -> mPeriodicIO.turnPosition);
    }
    
    public void setDesiredState(ModuleState desiredState, boolean isOpenLoop) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001 && !isOpenLoop) {
            stop();
            return;

        } else {
            
            // Apply chassis angular offset to the desired state.
            ModuleState optimizedDesiredState = ModuleState.fromSpeeds(
                desiredState.angle.plus(edu.wpi.first.math.geometry.Rotation2d.fromRadians(mConstants.kOffset)), 
                Util.limit(desiredState.speedMetersPerSecond, Constants.DriveConstants.kMaxVel));

            optimizedDesiredState = ModuleState.optimize(optimizedDesiredState, Rotation2d.fromRadians(mTurnEncoder.getPosition()).toWPI());

            mPeriodicIO.driveDemand = optimizedDesiredState.speedMetersPerSecond;
            mPeriodicIO.rotationDemand = optimizedDesiredState.angle.getRadians();

        }
    }


    public void stop() {
        mPeriodicIO.driveDemand = 0.0;
        mPeriodicIO.rotationDemand = mPeriodicIO.turnPosition;

        mDrive.setVoltage(0.0);
        mTurn.setVoltage(0.0);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public ModuleState getState() {
        return new ModuleState(
                mPeriodicIO.drivePosition,
                Rotation2d.fromRadians(mPeriodicIO.turnPosition - mConstants.kOffset).toWPI(),
                mPeriodicIO.driveVelocity);
    }

    /** Zeroes SwerveModule drive encoder. */
    public void resetEncoders() {
        mDriveEncoder.setPosition(0);
    }

    private class PeriodicIO {
        // inputs
        double drivePosition = 0.0;
        double turnPosition = 0.0;

        double driveVelocity = 0.0;

        // outputs
        double driveDemand = 0.0;
        double rotationDemand = 0.0;
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.drivePosition = mDriveEncoder.getPosition();
        mPeriodicIO.turnPosition = mTurnEncoder.getPosition();

        mPeriodicIO.driveVelocity = mDriveEncoder.getVelocity();
    }

    @Override
    public void writePeriodicOutputs() {
        mDrive.setReference(mPeriodicIO.driveDemand, ControlType.kVelocity, 0, 0, ArbFFUnits.kVoltage);
        mTurn.setReference(mPeriodicIO.rotationDemand, ControlType.kPosition, 0, 0, ArbFFUnits.kVoltage);
    }

    @Override
    public void outputTelemetry() {
        if(Constants.disableExtraTelemetry) return;

    }
}
