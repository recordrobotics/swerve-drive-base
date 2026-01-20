package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.real.NavSensorReal;
import frc.robot.subsystems.io.sim.NavSensorSim;
import frc.robot.utils.DriverStationUtils;
import java.util.Optional;

public class PoseSensorFusion extends SubsystemBase {

    public static final double MAX_MEASUREMENT_STD_DEVS = 9_999_999;

    private static final double ISPE_STD_DEV = 0.7;

    private static final double DEFAULT_DEBOUNCE_TIME = 0.5;

    public final NavSensor nav;

    private SwerveDrivePoseEstimator poseFilter;
    // private IndependentSwervePoseEstimator independentPoseEstimator;

    private boolean useISPE;

    private double updateTimestamp;
    private Rotation2d updateNav;
    private SwerveModulePosition[] updatePositions;

    public PoseSensorFusion() {
        nav = new NavSensor(
                Constants.RobotState.getMode() == Mode.REAL
                        ? new NavSensorReal()
                        : new NavSensorSim(RobotContainer.drivetrain
                                .getSwerveDriveSimulation()
                                .getGyroSimulation()));
        nav.resetAngleAdjustment();

        poseFilter = new SwerveDrivePoseEstimator(
                RobotContainer.drivetrain.getKinematics(),
                nav.getAdjustedAngle(),
                getModulePositions(),
                DashboardUI.Overview.getStartingLocation().getPose());

        // independentPoseEstimator = new IndependentSwervePoseEstimator(
        //         getEstimatedPosition(),
        //         new SwerveModule[] {
        //             RobotContainer.drivetrain.getFrontLeftModule(),
        //             RobotContainer.drivetrain.getFrontRightModule(),
        //             RobotContainer.drivetrain.getBackLeftModule(),
        //             RobotContainer.drivetrain.getBackRightModule()
        //         },
        //         new Translation2d[] {
        //             Constants.Swerve.FRONT_LEFT_WHEEL_LOCATION,
        //             Constants.Swerve.FRONT_RIGHT_WHEEL_LOCATION,
        //             Constants.Swerve.BACK_LEFT_WHEEL_LOCATION,
        //             Constants.Swerve.BACK_RIGHT_WHEEL_LOCATION
        //         });

        SmartDashboard.putBoolean("Overview/UseISPE", true);
    }

    public void startCalculation() {
        useISPE = SmartDashboard.getBoolean("Overview/UseISPE", true);

        calculationLoop();
    }

    public void endCalculation() {
        if (updatePositions != null) {
            poseFilter.updateWithTime(updateTimestamp, updateNav, updatePositions);
        }

        updateDashboard();

        // Logger.recordOutput("SwerveEstimations", independentPoseEstimator.getEstimatedModulePositions());
        // Logger.recordOutput("RobotEstimations", independentPoseEstimator.getEstimatedRobotPoses());
        // Logger.recordOutput("RobotEstimation", independentPoseEstimator.getEstimatedRobotPose());
    }

    @Override
    public void periodic() {
        /* logic is asynchronous */
    }

    public void calculationLoop() {
        updateTimestamp = Timer.getTimestamp();
        updateNav = nav.getAdjustedAngle();
        updatePositions = getModulePositions();

        // independentPoseEstimator.update(getEstimatedPosition().getRotation());

        // // when no vision use independent pose estimator to correct pose
        // if (useISPE) {
        //     poseFilter.addVisionMeasurement(
        //             independentPoseEstimator.getEstimatedRobotPose(),
        //             Timer.getTimestamp(),
        //             VecBuilder.fill(ISPE_STD_DEV, ISPE_STD_DEV, MAX_MEASUREMENT_STD_DEVS));
        // }
    }

    private void updateDashboard() {
        SmartDashboard.putNumber(
                "pose", poseFilter.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("gyro", nav.getAdjustedAngle().getDegrees());
        DashboardUI.Overview.setRobotPose(poseFilter.getEstimatedPosition());
    }

    private static SwerveModulePosition[] getModulePositions() {
        return RobotContainer.drivetrain.getModulePositions();
    }

    public Pose2d getEstimatedPosition() {
        return poseFilter.getEstimatedPosition();
    }

    public Pose2d getEstimatedPositionAt(double timestamp) {
        Optional<Pose2d> sample = poseFilter.sampleAt(timestamp);
        if (sample.isEmpty()) return getEstimatedPosition();
        else return sample.get();
    }

    /** Similar to resetPose but adds an argument for the initial pose */
    public void setToPose(Pose2d pose) {
        poseFilter.resetPosition(nav.getAdjustedAngle(), getModulePositions(), pose);
        // independentPoseEstimator.reset(pose);
    }

    /** Resets the field relative position of the robot (mostly for testing). */
    public void resetStartingPose() {
        setToPose(DashboardUI.Overview.getStartingLocation().getPose());
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            RobotContainer.drivetrain
                    .getSwerveDriveSimulation()
                    .setSimulationWorldPose(
                            DashboardUI.Overview.getStartingLocation().getPose());
        }
    }

    /**
     * Resets the pose to face elevator away from driverstation, while keeping translation the same
     */
    public void resetDriverPose() {
        Pose2d current = getEstimatedPosition();
        setToPose(new Pose2d(
                current.getTranslation(),
                DriverStationUtils.getCurrentAlliance() == Alliance.Red ? new Rotation2d(Math.PI) : new Rotation2d(0)));
    }
}
