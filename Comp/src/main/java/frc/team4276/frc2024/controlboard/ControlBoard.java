package frc.team4276.frc2024.controlboard;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Ports;
import frc.team4276.frc2024.field.AllianceChooser;
import frc.team4276.frc2024.Constants.OIConstants;
import frc.team4276.frc2024.subsystems.ClimberSubsystem;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.frc2024.subsystems.FourbarSubsystem;
import frc.team4276.frc2024.subsystems.IntakeSubsystem;
import frc.team4276.frc2024.subsystems.Superstructure;

import frc.team1678.lib.Util;
import frc.team1678.lib.swerve.ChassisSpeeds;

import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;

public class ControlBoard {
    public final BetterXboxController driver;
    public final BetterXboxController operator;

    private final DigitalInput climberSetting;
    private final DigitalInput fourbarSetting;

    private DriveSubsystem mDriveSubsystem;
    private Superstructure mSuperstructure;
    private ClimberSubsystem mClimberSubsystem;

    private static ControlBoard mInstance;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    private ControlBoard() {
        driver = new BetterXboxController(OIConstants.kDriverControllerPort);
        operator = new BetterXboxController(OIConstants.kOpControllerPort);

        climberSetting = new DigitalInput(Ports.CLIMBER_BRAKE_SWITCH);
        fourbarSetting = new DigitalInput(Ports.FOURBAR_BRAKE_SWITCH);

        mDriveSubsystem = DriveSubsystem.getInstance();
        mSuperstructure = Superstructure.getInstance();
        mClimberSubsystem = ClimberSubsystem.getInstance();
    }

    private double mTuningFourbarVoltage = 0.0;
    private double mTuningFourbarSetpoint = 90.0;

    public void updateTuning() {
        mSuperstructure.setManual(false);

        double sign = driver.getYButton() ? -1 : 1;

        if (driver.getRightBumperReleased()) {
            mTuningFourbarVoltage += 0.01 * sign;
        }

        if (driver.getLeftBumperReleased()) {
            mTuningFourbarVoltage += 0.1 * sign;
        }

        SmartDashboard.putNumber("Debug/Test/Tuning Fourbar Voltage", mTuningFourbarVoltage);

        // mSuperstructure.setManualFourbarVoltage(mTuningFourbarVoltage);
        
        if (driver.getXButtonReleased()) {
            mTuningFourbarSetpoint += 1 * sign;
        }

        if (driver.getBButtonReleased()) {
            mTuningFourbarSetpoint += 10 * sign;
        }

        SmartDashboard.putNumber("Debug/Test/Tuning Fourbar Setpoint", mTuningFourbarSetpoint);


        FourbarSubsystem.getInstance().setFuseMotionSetpoint(mTuningFourbarSetpoint);

        // mFourbarSubsystem.setFuseMotionSetpoint(SmartDashboard.getNumber("Debug/Test/Fourbar
        // Des Position", 90.0));

        // val = SmartDashboard.getNumber("Debug/Test/Desired Fourbar Voltage",
        // Double.NaN);

        // if (val != Double.NaN) {
        // mFourbarSubsystem.setVoltage(val);
        // }

        // mFlywheelSubsystem.setTargetRPM(SmartDashboard.getNumber("Debug/Test/Flywheel
        // Des RPM", 0.0));
    }

    public void update() {
        driver.update();
        operator.update();

        if (wantDemoLimits()) {
            mDriveSubsystem.setKinematicLimits(Constants.DriveConstants.kDemoLimits);
        } else {
            mDriveSubsystem.setKinematicLimits(Constants.DriveConstants.kUncappedLimits);
        }

        if (wantZeroHeading()) {
            mDriveSubsystem.resetGyro(AllianceChooser.getInstance().isAllianceRed() ? 180.0 : 0.0);
        }

        if (wantXBrake()) {
            mDriveSubsystem.setX();
        } else {
            mDriveSubsystem.teleopDrive(ChassisSpeeds.fromFieldRelativeSpeeds(
                    getSwerveTranslation().x(),
                    getSwerveTranslation().y(),
                    getSwerveRotation(),
                    mDriveSubsystem.getHeading().toWPI(),
                    AllianceChooser.getInstance().isAllianceRed()));
        }

        if (wantManual()) {
            mSuperstructure.setManual(true);
            updateManual();

        } else {
            mSuperstructure.setManual(false);
            updateNominal();

        }

        if (wantRaiseClimber()) {
            mClimberSubsystem.setDesiredState(ClimberSubsystem.State.RAISE);

        } else if (wantSlowLowerClimber() && wantClimbMode()) {
            mSuperstructure.setForceDisablePrep(true);
            mClimberSubsystem.setDesiredState(ClimberSubsystem.State.SLOW_LOWER);

        } else if (wantLowerClimber() && wantClimbMode()) {
            mSuperstructure.setForceDisablePrep(true);
            mClimberSubsystem.setDesiredState(ClimberSubsystem.State.LOWER);

        } else if (!wantClimbMode()) {
            mSuperstructure.setForceDisablePrep(false);
            mClimberSubsystem.setDesiredState(ClimberSubsystem.State.IDLE);

        } else {
            mClimberSubsystem.setDesiredState(ClimberSubsystem.State.IDLE);

        }
    }

    public void updateNominal() {
        mDriveSubsystem.overrideHeading(wantReady());

        mSuperstructure.setDynamic(wantDynamic());

        mSuperstructure.setFerry(wantFerry());

        if (wantOffsetFerry()) {
            if (wantIncrementOffset()) {
                mSuperstructure.offsetFerry(1.0);
            } else if (wantDecrementOffset()) {
                mSuperstructure.offsetFerry(-1.0);
            }
        }

        if (wantOffsetScoring()) {
            if (wantIncrementOffset()) {
                mSuperstructure.offsetScoring(1.0);
            } else if (wantDecrementOffset()) {
                mSuperstructure.offsetScoring(-1.0);
            }
        }

        // If vision is disabled
        if (wantStow()) {
            mSuperstructure.setPrep(false);
        } else if (wantPrep()) {
            mSuperstructure.setPrep(true);
        }

        if (wantIdle()) {
            mSuperstructure.setGoalState(Superstructure.GoalState.IDLE);

        } else if (wantIntake()) {
            mSuperstructure.setGoalState(Superstructure.GoalState.INTAKE);

        } else if (wantShoot()) {
            mSuperstructure.setGoalState(Superstructure.GoalState.SHOOT);

        } else if (wantReady()) {
            mSuperstructure.setGoalState(Superstructure.GoalState.READY);

        } else if (wantExhaust()) {
            mSuperstructure.setGoalState(Superstructure.GoalState.EXHAUST);

        } else {
            mSuperstructure.setGoalState(Superstructure.GoalState.STOW);

        }
    }

    public void updateManual() {
        if (wantManualReadyFlywheel()) {
            mSuperstructure.setManualFlywheelVoltage(11.0);
        } else if (wantManualSpinup()) {
            mSuperstructure.setManualFlywheelVoltage(Constants.FlywheelConstants.kPrep);
        } else {
            mSuperstructure.setManualFlywheelVoltage(0.0);
        }

        mSuperstructure.setManualFourbarVoltage(operator.getRightYDeadband() * 3.0);

        if (wantManualIntake()) {
            mSuperstructure.setManualIntakeState(IntakeSubsystem.State.INTAKE);
        } else if (wantManualShoot()) {
            mSuperstructure.setManualIntakeState(IntakeSubsystem.State.SHOOT);
        } else if (wantManualExhaust()) {
            mSuperstructure.setManualIntakeState(IntakeSubsystem.State.EXHAUST);
        } else if (wantManualDefeed()) {
            mSuperstructure.setManualIntakeState(IntakeSubsystem.State.DEFEED);
        } else if (wantManualSlowFeed()) {
            mSuperstructure.setManualIntakeState(IntakeSubsystem.State.SLOW_FEED);
        } else {
            mSuperstructure.setManualIntakeState(IntakeSubsystem.State.IDLE);
        }
    }

    // Driver Controls
    public Translation2d getSwerveTranslation() {
        double forwardAxis = -driver.getLeftY();
        double strafeAxis = -driver.getLeftX();

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.norm()) < OIConstants.kJoystickDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(tAxes.x(), tAxes.y(), true);
            Translation2d deadband_vector = Translation2d.fromPolar(deadband_direction, OIConstants.kJoystickDeadband);

            double scaled_x = Util.scaledDeadband(forwardAxis, 1.0, Math.abs(deadband_vector.x()));
            double scaled_y = Util.scaledDeadband(strafeAxis, 1.0, Math.abs(deadband_vector.y()));
            return new Translation2d(scaled_x, scaled_y)
                    .scale(DriveSubsystem.getInstance().getKinematicLimits().kMaxDriveVelocity);
        }
    }

    private double kMaxTurnMagnitude = 0.75;

    public double getSwerveRotation() {
        double rotAxis = -driver.getRightX();

        if (Math.abs(rotAxis) < OIConstants.kJoystickDeadband) {
            return 0.0;
        } else {
            return kMaxTurnMagnitude * DriveSubsystem.getInstance().getKinematicLimits().kMaxAngularVelocity
                    * (rotAxis - (Math.signum(rotAxis) * OIConstants.kJoystickDeadband))
                    / (1 - OIConstants.kJoystickDeadband);
        }
    }

    public boolean wantZeroHeading() {
        return driver.getAButtonPressed();
    }

    public boolean wantXBrake() {
        return driver.getXButton();
    }

    boolean isDemo = false;
    boolean hasReleased = false;

    public boolean wantDemoLimits() {
        if (!driver.getPOVUP()) {
            hasReleased = true;

        }

        if (driver.getPOVUP() && hasReleased) {
            hasReleased = false;
            isDemo = !isDemo;
        }

        return isDemo;
    }

    public boolean wantIntake() {
        return driver.getLT();
    }

    public boolean wantExhaust() {
        return driver.getLeftBumper();
    }

    public boolean wantShoot() {
        return driver.getYButton();
    }

    public boolean wantReady() {
        return driver.getRT();
    }

    public boolean wantLowerClimber() {
        return driver.getRightBumper();
    }

    // Operator Controls
    public boolean wantStow() {
        return false;
    }

    public boolean wantPrep() {
        return false;
    }

    public boolean wantDynamic() {
        return false;
    }

    public boolean wantFerry() {
        return false;
    }

    private boolean wasIdle = false;

    public boolean wantIdle() {
        if (operator.getBButtonPressed()) {
            wasIdle = !wasIdle;
        }

        return wasIdle;
    }

    public boolean wantOffsetFerry() {
        return false;
    }

    public boolean wantOffsetScoring() {
        return false;
    }

    public boolean wantIncrementOffset() {
        return operator.getPOVUP();
    }

    public boolean wantDecrementOffset() {
        return operator.getPOVDOWN();
    }

    public boolean wantManual() {
        return true;
    }

    public boolean wantManualSpinup() {
        return false;
    }

    public boolean wantManualReadyFlywheel() {
        return operator.getLT();
    }

    public boolean wantManualIntake() {
        return false;
    }

    public boolean wantManualShoot() {
        return operator.getRT();
    }

    public boolean wantManualExhaust() {
        return operator.getXButton();
    }

    public boolean wantManualDefeed() {
        return false;
    }

    public boolean wantManualSlowFeed() {
        return false;
    }

    public boolean wantClimbMode() {
        return false;
    }

    public boolean wantRaiseClimber() {
        return false;
    }

    public boolean wantSlowLowerClimber() {
        return false;
    }

    // Robot Button Board
    public boolean wantClimberCoastMode() {
        return climberSetting.get();
    }

    public boolean wantFourbarCoastMode() {
        return fourbarSetting.get();
    }

}