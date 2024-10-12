package frc.team4276.frc2024.controlboard;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Ports;
import frc.team4276.frc2024.field.AllianceChooser;
import frc.team4276.frc2024.Constants.OIConstants;
// import frc.team4276.frc2024.subsystems.ClimberSubsystem;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
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
    // private ClimberSubsystem mClimberSubsystem;

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
        // mClimberSubsystem = ClimberSubsystem.getInstance();
    }

    private double mTuningFlywheelSetpoint = 4500;
    private double mTuningFourbarSetpoint = 90.0;

    public void updateTuning() {
        mSuperstructure.setTuning();

        double sign = operator.getYButton() ? -1 : 1;

        if (operator.getRightBumperReleased()) {
            mTuningFlywheelSetpoint += 100 * sign;
        }

        if (operator.getLeftBumperReleased()) {
            mTuningFlywheelSetpoint += 1000 * sign;
        }

        // SmartDashboard.putNumber("Debug/Test/Tuning Flywheel Setpoint", mTuningFlywheelSetpoint);
      
        if (operator.getLT()) {
            mSuperstructure.setTuningFlywheelRPM(mTuningFlywheelSetpoint);
        } else {
            mSuperstructure.setTuningFlywheelRPM(0.0);
        }
        
        if (operator.getXButtonReleased()) {
            mTuningFourbarSetpoint += 1 * sign;
        }

        if (operator.getBButtonReleased()) {
            mTuningFourbarSetpoint += 10 * sign;
        }

        // SmartDashboard.putNumber("Debug/Test/Tuning Fourbar Setpoint", mTuningFourbarSetpoint);

        mSuperstructure.setTuningFourbarPostion(mTuningFourbarSetpoint);

        if (operator.getRT()) {
            mSuperstructure.setTuningIntakeState(IntakeSubsystem.State.SHOOT);
        } else {
            mSuperstructure.setTuningIntakeState(IntakeSubsystem.State.IDLE);
        }
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
            updateManual();

        } else {
            updateNominal();

        }

        // if (wantRaiseClimber()) {
        //     // mClimberSubsystem.setDesiredState(ClimberSubsystem.State.RAISE);

        // } else if (wantSlowLowerClimber() && wantClimbMode()) {
        //     mSuperstructure.setForceDisablePrep(true);
        //     // mClimberSubsystem.setDesiredState(ClimberSubsystem.State.SLOW_LOWER);

        // } else if (wantLowerClimber() && wantClimbMode()) {
        //     mSuperstructure.setForceDisablePrep(true);
        //     // mClimberSubsystem.setDesiredState(ClimberSubsystem.State.LOWER);

        // } else if (!wantClimbMode()) {
        //     mSuperstructure.setForceDisablePrep(false);
        //     // mClimberSubsystem.setDesiredState(ClimberSubsystem.State.IDLE);

        // } else {
        //     // mClimberSubsystem.setDesiredState(ClimberSubsystem.State.IDLE);

        // }
    }

    public void updateNominal() {
        mSuperstructure.setNominal();

        mDriveSubsystem.overrideHeading(wantReady() && wantDynamic());

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

        } else if (wantPoop()) {
            mSuperstructure.setGoalState(Superstructure.GoalState.POOP);

        } else {
            mSuperstructure.setGoalState(Superstructure.GoalState.STOW);

        }
    }

    public void updateManual() {
        mSuperstructure.setManual();

        if (wantManualReadyFlywheel()) {
            mSuperstructure.setManualFlywheelVoltage(11.0);
        } else if (wantManualSpinup()) {
            mSuperstructure.setManualFlywheelVoltage(Constants.FlywheelConstants.kPrep);
        } else if (wantManualExhaust()) {
            mSuperstructure.setManualFlywheelVoltage(-3.0);
        } else {
            mSuperstructure.setManualFlywheelVoltage(0.0);
        }

        mSuperstructure.setManualFourbarVoltage(operator.getRightYDeadband() * -3.0);

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
        return driver.getLeftBumper() || operator.getXButton();
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

    public boolean wantPoop() {
        return false;
    }

    // Operator Controls
    public boolean wantStow() {
        return operator.getAButton();
    }

    public boolean wantPrep() {
        return operator.getLT();
    }

    public boolean wantDynamic() {
        return true;
    }

    public boolean wantFerry() {
        return operator.getBButton();
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

    // Extra
    public boolean enableFourbarFuse() {
        return true;
    }

}