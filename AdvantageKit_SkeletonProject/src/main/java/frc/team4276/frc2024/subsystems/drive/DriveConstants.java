package frc.team4276.frc2024.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2024.Ports;
import frc.team4276.frc2024.subsystems.drive.ModuleIOSparkMax.ModuleConfig;
import frc.team4276.lib.rev.SparkMaxFactory;

public class DriveConstants {
    public static final int kDrivingMotorPinionTeeth = 13;

    public static final double kWheelCircumferenceMeters = Units.inchesToMeters(3.0) * Math.PI;

    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (5676 / 60 * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = kWheelCircumferenceMeters
            / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = (kWheelCircumferenceMeters
            / kDrivingMotorReduction) / 60.0; // meters per second

    public static final SparkMaxFactory.CANSparkMaxPIDFConfig kDrivingPIDFConfig = new SparkMaxFactory.CANSparkMaxPIDFConfig();
    static {
        kDrivingPIDFConfig.kP = 0.04;
        kDrivingPIDFConfig.kFF = 1 / kDriveWheelFreeSpeedRps;
    }

    public static final SparkMaxFactory.CANSparkMaxPIDFConfig kTurningPIDFConfig = new SparkMaxFactory.CANSparkMaxPIDFConfig();
    static {
        kTurningPIDFConfig.kP = 1.0;
    } 

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps

    public static final double kTrackWidthX = Units.inchesToMeters(23.5);
    public static final double kTrackWidthY = Units.inchesToMeters(23.5);
    
    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            new Translation2d(kTrackWidthY / 2, kTrackWidthX / 2),
            new Translation2d(kTrackWidthY / 2, -kTrackWidthX / 2),
            new Translation2d(-kTrackWidthY / 2, kTrackWidthX / 2),
            new Translation2d(-kTrackWidthY / 2, -kTrackWidthX / 2));

    public static ModuleConfig[] kModuleConfigs = new ModuleConfig[4];
    static {
        kModuleConfigs[0] = new ModuleConfig();
        kModuleConfigs[0].kName = "Front Left";
        kModuleConfigs[0].kDriveId = Ports.FRONT_LEFT_DRIVE;
        kModuleConfigs[0].kTurnId = Ports.FRONT_LEFT_TURN;
        kModuleConfigs[0].kOffset = -Math.PI / 2;
        
        kModuleConfigs[1] = new ModuleConfig();
        kModuleConfigs[1].kName = "Front Right";
        kModuleConfigs[1].kDriveId = Ports.FRONT_RIGHT_DRIVE;
        kModuleConfigs[1].kTurnId = Ports.FRONT_RIGHT_TURN;
        kModuleConfigs[1].kOffset = 0;

        kModuleConfigs[2] = new ModuleConfig();
        kModuleConfigs[2].kName = "Back Left";
        kModuleConfigs[2].kDriveId = Ports.BACK_LEFT_DRIVE;
        kModuleConfigs[2].kTurnId = Ports.BACK_LEFT_TURN;
        kModuleConfigs[2].kOffset = Math.PI;
        
        kModuleConfigs[3] = new ModuleConfig();
        kModuleConfigs[3].kName = "Back Right";
        kModuleConfigs[3].kDriveId = Ports.BACK_RIGHT_DRIVE;
        kModuleConfigs[3].kTurnId = Ports.BACK_RIGHT_TURN;
        kModuleConfigs[3].kOffset = Math.PI / 2;
    }

    public static final double kMaxVel = kDriveWheelFreeSpeedRps * 0.8; // meters per second
    public static final double kMaxAccel = 8.9;
    public static final double kMaxAngularVel = kMaxVel * 2 / (kTrackWidthX * Math.sqrt(2)); // radians per second
    public static final double kMaxAngularAccel = 30.0;

    public static final double kAutoTranslationKp = 3.0;
    public static final double kAutoTranslationKd = 0.0;
    public static final double kAutoRotationKp = 4.0;
    public static final double kAutoRotationKd = 0.0;

    public static final double kAutoTransAccelFF = 0.0;
    public static final double kAutoRotAccelFF = 0.0;

    public static final double kAutoMaxError = 0.75; // Meters

    public static final double kSnapHeadingKp = 3.0;
    public static final double kSnapHeadingKi = 0.0;
    public static final double kSnapHeadingKd = 0.0;

    public static final double kSnapPositionTolerance = 0.1;
    public static final double kSnapAngularVelocityTolerance = 0.1;
    
    public static final double kProfiledSnapHeadingKp = 0.0;
    public static final double kProfiledSnapHeadingKi = 0.0;
    public static final double kProfiledSnapHeadingKd = 0.0;

    public static final double kProfiledSnapPositionTolerance = 2 * Math.PI / 180;
    public static final double kProfiledSnapAngularVelocityTolerance = 0.5;
}
