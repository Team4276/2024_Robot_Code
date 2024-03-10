package frc.team4276.frc2024.auto.modes;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.PPSwerveTrajectoryAction;
import frc.team4276.frc2024.auto.actions.WaitAction;
import frc.team4276.frc2024.statemachines.FlywheelState;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.subsystems.FlywheelSubsystem.DesiredFlywheelMode;
import frc.team4276.frc2024.subsystems.IntakeSubsystem.IntakeState;

public class SubAmpSide2Piece extends AutoModeBase {
    private Superstructure mSuperstructure = Superstructure.getInstance();

    private final PPSwerveTrajectoryAction traj1;
    private final PPSwerveTrajectoryAction traj2;

    public SubAmpSide2Piece() {
        traj1 = new PPSwerveTrajectoryAction("SubASCtoAmpPickup1");
        traj2 = new PPSwerveTrajectoryAction("AmpPickup1toSubASC");
    }

    @Override
    protected void routine() throws AutoModeEndedException { 
        // Shoot preload
        mSuperstructure.setFourBarVoltage(-2.0);
        runAction(new WaitAction(2.0));
        mSuperstructure.setFourBarVoltage(0.0);
        mSuperstructure.setFlywheelState(new FlywheelState(DesiredFlywheelMode.RPM, -3500, -3500));
        runAction(new WaitAction(2.0));
        mSuperstructure.setIntakeState(IntakeState.FOOT);
        runAction(new WaitAction(2.0));
        mSuperstructure.setIntakeState(IntakeState.IDLE);

        // Drive to note and intake
        mSuperstructure.setIntakeState(IntakeState.FASTAKE);
        mSuperstructure.setFlywheelState(new FlywheelState());
        runAction(traj1);

        // Drive to Sub and shoot
        runAction(traj2);
        mSuperstructure.setFlywheelState(new FlywheelState(DesiredFlywheelMode.RPM, -3500, -3500));
        runAction(new WaitAction(2.0));
        mSuperstructure.setIntakeState(IntakeState.FOOT);
    }

    @Override
    public Pose2d getStartingPose() {
        return traj1.getInitialPose();
    }
}
