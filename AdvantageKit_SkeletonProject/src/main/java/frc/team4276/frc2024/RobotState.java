package frc.team4276.frc2024;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import frc.team4276.frc2024.field.Field;
import frc.team4276.frc2024.subsystems.drive.DriveConstants;
import frc.team4276.lib.MovingAverage;

public class RobotState {
    private boolean kVisionResetsHeading = false; //TODO: impl auto heading reset with reliable reads

    private MovingAverage mEstimatedVisionHeading = new MovingAverage(100);

    private ExtendedKalmanFilter<N2, N2, N2> mKalmanFilter;
    private final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(Math.pow(0.05, 1), Math.pow(0.05, 1));
    private static final Matrix<N2, N1> kLocalMeasurementStdDevs = VecBuilder.fill(Math.pow(0.03, 1),
                Math.pow(0.03, 1));
    private boolean mHasUpdated = false;

    private static final double kObservationBufferTime = 1.0;
    
    private Pose2d mEstimatedPose = new Pose2d();

    private final TimeInterpolatableBuffer<Pose2d> mOdomPoseBuffer;
    private Pose2d mOdomPose = new Pose2d();
    private SwerveDriveWheelPositions mPrevWheelPositions = new SwerveDriveWheelPositions(
        new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        });
    private Rotation2d mPrevGyroHeading = new Rotation2d();

    private Field.POIs mPOIs = Field.Red.kPOIs;

    private InterpolatingDoubleTreeMap kSpeakerFourbarAngles = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap kSpeakerFlywheelRPMs = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap kFerryFourbarAngles = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap kFerryFlywheelRPMs = new InterpolatingDoubleTreeMap();

    public record AimingParameters(
            Rotation2d driveHeading,
            double fourbarSetpoint,
            double flywheelRpm,
            double distance) {

        public Rotation2d getDriveHeading() {
            return driveHeading;
        }

        public double getFourbarSetpoint() {
            return fourbarSetpoint;
        }

        public double getFlywheelRpm() {
            return flywheelRpm;
        }

        public double getDistance() {
            return distance;
        }
    }

    private AimingParameters latestSpeakerParams;
    private AimingParameters latestFerryParams;

    private static RobotState mInstance;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }
        return mInstance;
    }

    private RobotState() {
        mOdomPoseBuffer = TimeInterpolatableBuffer.createBuffer(kObservationBufferTime);

        reset(0.0, new Pose2d());
        
        kSpeakerFourbarAngles.put(1.0, 135.0);
        kSpeakerFourbarAngles.put(1.5, 130.0);
        kSpeakerFourbarAngles.put(2.0, 120.0);
        kSpeakerFourbarAngles.put(2.25, 125.0);
        kSpeakerFourbarAngles.put(2.5, 110.0);
        // kSpeakerFourbarAngles.put(2.75, 110.0);
        kSpeakerFourbarAngles.put(3.0, 106.5);
        // kSpeakerFourbarAngles.put(3.25, 106.5);
        kSpeakerFourbarAngles.put(3.5, 105.0);
        // kSpeakerFourbarAngles.put(3.75, 105.0);
        kSpeakerFourbarAngles.put(4.0, 104.0);
        // kSpeakerFourbarAngles.put(4.25, 104.0);
        kSpeakerFourbarAngles.put(4.5, 103.0);
        // kSpeakerFourbarAngles.put(4.75, 102.0);
        kSpeakerFourbarAngles.put(5.0, 102.0);
        kSpeakerFourbarAngles.put(5.5, 101.0);
        kSpeakerFourbarAngles.put(6.0, 100.0);
        kSpeakerFourbarAngles.put(6.5, 95.0);
        kSpeakerFourbarAngles.put(7.0, 90.0);

        kSpeakerFlywheelRPMs.put(1.0, 3500.0);
        kSpeakerFlywheelRPMs.put(1.5, 4000.0);
        kSpeakerFlywheelRPMs.put(2.0, 4000.0);
        kSpeakerFlywheelRPMs.put(2.5, 4500.0);
        kSpeakerFlywheelRPMs.put(3.0, 5000.0);
        kSpeakerFlywheelRPMs.put(3.5, 5000.0);
        kSpeakerFlywheelRPMs.put(4.0, 5000.0);
        kSpeakerFlywheelRPMs.put(4.5, 5000.0);
        kSpeakerFlywheelRPMs.put(5.0, 5000.0);
        kSpeakerFlywheelRPMs.put(5.5, 5000.0);
        kSpeakerFlywheelRPMs.put(6.0, 5000.0);
        kSpeakerFlywheelRPMs.put(6.5, 5000.0);
        kSpeakerFlywheelRPMs.put(7.0, 5000.0);

        kFerryFourbarAngles.put(5.3, 130.0);
        kFerryFourbarAngles.put(7.9, 130.0);
        kFerryFourbarAngles.put(9.3, 130.0);
        kFerryFourbarAngles.put(12.3, 135.0);

        kFerryFlywheelRPMs.put(5.3, 3500.0);
        kFerryFlywheelRPMs.put(7.9, 4000.0);
        kFerryFlywheelRPMs.put(9.3, 4500.0);
        kFerryFlywheelRPMs.put(12.3, 5000.0);

        resetKalmanFilters();
    }

    public synchronized void reset(double start_time, Pose2d initial_pose) { //TODO: check if need to reset prev heading
        mOdomPose = initial_pose;
        mEstimatedPose = initial_pose;
        mOdomPoseBuffer.clear();
        mOdomPoseBuffer.addSample(start_time, initial_pose);
    }

    public synchronized void resetHeading(double start_time, Rotation2d currentHeading) {
        reset(start_time, new Pose2d(mEstimatedPose.getTranslation(), currentHeading));
        mPrevGyroHeading = mPrevGyroHeading.plus(currentHeading.minus(mEstimatedPose.getRotation()));
    }


    public synchronized void resetKalmanFilters() {
        mKalmanFilter = new ExtendedKalmanFilter<N2, N2, N2>(
                Nat.N2(), // Dimensions of output (x, y)
                Nat.N2(), // Dimensions of predicted error shift (dx, dy) (always 0)
                Nat.N2(), // Dimensions of vision (x, y)
                (x, u) -> u, // The derivative of the output is predicted shift (always 0)
                (x, u) -> x, // The output is position (x, y)
                kStateStdDevs, // Standard deviation of position (uncertainty propagation
                // with no vision)
                kLocalMeasurementStdDevs, // Standard deviation of vision measurements
                Constants.kLooperDt);
    }

    public synchronized Field.POIs getPOIs() {
        return mPOIs;
    }

    public synchronized void setBlue() {
        mPOIs = Field.Blue.kPOIs;
    }

    public synchronized void setRed() {
        mPOIs = Field.Red.kPOIs;
    }

    public synchronized void addOdomObservations(double timestamp, SwerveDriveWheelPositions positions,
            Rotation2d heading) {
        latestSpeakerParams = null;
        latestFerryParams = null;

        Twist2d twist = DriveConstants.kKinematics.toTwist2d(mPrevWheelPositions, positions);
        twist.dtheta = heading.minus(mPrevGyroHeading).getRadians();

        mOdomPose = mOdomPose.exp(twist);
        mOdomPoseBuffer.addSample(timestamp, mOdomPose);

        mEstimatedPose = mEstimatedPose.exp(twist);
        
        mPrevWheelPositions = positions;
        mPrevGyroHeading = heading;

        mKalmanFilter.predict(VecBuilder.fill(0.0, 0.0), Constants.kLooperDt);
    }

    public static class VisionUpdate {
        public final double timestamp;
        public final Pose2d fieldToVis;
        public final double distStDev;

        public VisionUpdate(double timestamp, Pose2d fieldToVis, double distStDev) {
            this.timestamp = timestamp;
            this.fieldToVis = fieldToVis;
            this.distStDev = distStDev;
        }
    }

    public synchronized void visionUpdate(VisionUpdate update) {
        double visionTimestamp = update.timestamp;

        if (mOdomPoseBuffer.getInternalBuffer().lastKey() - kObservationBufferTime > visionTimestamp)
            return;

        mEstimatedVisionHeading.addNumber(update.fieldToVis.getRotation().getRadians());

        new Transform2d(mOdomPose, mEstimatedPose);
        
        mEstimatedPose = new Pose2d(update.fieldToVis.getTranslation().plus(
            mOdomPoseBuffer.getInternalBuffer().lastEntry().getValue().getTranslation()).
            minus(mOdomPoseBuffer.getSample(visionTimestamp).get().getTranslation()), 
            new Rotation2d());

        if (!mHasUpdated) {
            mKalmanFilter.setXhat(0, mEstimatedPose.getX());
            mKalmanFilter.setXhat(1, mEstimatedPose.getY());

            mHasUpdated = true;
            return;
        }

        try {
            mKalmanFilter.correct(VecBuilder.fill(0.0, 0.0),
                    VecBuilder.fill(mEstimatedPose.getX(), mEstimatedPose.getY()),
                    StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), VecBuilder.fill(update.distStDev, update.distStDev)));
            mEstimatedPose = new Pose2d(mKalmanFilter.getXhat(0), mKalmanFilter.getXhat(1), mEstimatedPose.getRotation());

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @AutoLogOutput(key = "RobotState/EstimatedPose")
    public synchronized Pose2d getEstimatedPose() {
        return mEstimatedPose;
    }

    public AimingParameters getSpeakerAimingParameters() {
        if (latestSpeakerParams != null) {
            return latestSpeakerParams; // return cached params if no new updates
        }

        Translation2d robot_to_target = getPOIs().kSpeakerCenter.minus(getEstimatedPose().getTranslation());

        double distance = robot_to_target.getNorm();
        double armAngle = kSpeakerFourbarAngles.get(distance);
        double flywheel_speeds = kSpeakerFlywheelRPMs.get(distance);

        latestSpeakerParams = new AimingParameters(robot_to_target.getAngle(), armAngle, flywheel_speeds, distance);

        return latestSpeakerParams;
    }

    public AimingParameters getFerryAimingParameters() {
        if (latestFerryParams != null) {
            return latestFerryParams; // return cached params if no new updates
        }

        Translation2d robot_to_target = getPOIs().kSpeakerCenter.minus(getEstimatedPose().getTranslation());

        double distance = robot_to_target.getNorm();
        double armAngle = kFerryFourbarAngles.get(distance);
        double flywheel_speeds = kFerryFlywheelRPMs.get(distance);

        latestFerryParams = new AimingParameters(robot_to_target.getAngle(), armAngle, flywheel_speeds, distance);

        return latestFerryParams;
    }

    /** Use on auton init */
    public synchronized double getHeadingFromVision() {
        if (mEstimatedVisionHeading.getSize() == 0) {
            return getEstimatedPose().getRotation().getRadians();
        }
        return mEstimatedVisionHeading.getAverage();
    }
}
