package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.SwerveModuleIO;
import frc.robot.subsystems.io.real.SwerveModuleReal;
import frc.robot.subsystems.io.sim.SwerveModuleSim;
import frc.robot.utils.ModuleConstants;
import frc.robot.utils.ModuleConstants.InvalidConfigException;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.modifiers.DrivetrainControl;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/** Represents a swerve drive style drivetrain. */
public final class Drivetrain extends SubsystemBase implements AutoCloseable {

    private static final int FL = 0;
    private static final int FR = 1;
    private static final int BL = 2;
    private static final int BR = 3;

    private static final double ENCODER_STABILIZATION_TIME = 2.3;

    private static final Velocity<VoltageUnit> SYSID_DRIVE_RAMP_RATE =
            Volts.of(3.0).per(Second);
    private static final Voltage SYSID_DRIVE_STEP_VOLTAGE = Volts.of(3.0);
    private static final Time SYSID_DRIVE_TIMEOUT = Seconds.of(1.5);

    private static final Velocity<VoltageUnit> SYSID_TURN_RAMP_RATE =
            Volts.of(6.0).per(Second);
    private static final Voltage SYSID_TURN_STEP_VOLTAGE = Volts.of(2.0);
    private static final Time SYSID_TURN_TIMEOUT = Seconds.of(1.0);

    // Creates swerve module objects
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final SysIdRoutine sysIdRoutineDriveMotorsSpin;
    private final SysIdRoutine sysIdRoutineDriveMotorsForward;
    private final SysIdRoutine sysIdRoutineTurnMotors;

    // Creates swerve kinematics
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            Constants.Swerve.FRONT_LEFT_WHEEL_LOCATION,
            Constants.Swerve.FRONT_RIGHT_WHEEL_LOCATION,
            Constants.Swerve.BACK_LEFT_WHEEL_LOCATION,
            Constants.Swerve.BACK_RIGHT_WHEEL_LOCATION);

    // Create and configure a drivetrain simulation configuration
    private final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
            // Specify gyro type (for realistic gyro drifting and error simulation)
            .withGyro(() -> new GyroSimulation(0.5, 0.05)) // navX-Micro
            // Specify swerve module (for realistic swerve dynamics)
            .withSwerveModule(new SwerveModuleSimulationConfig(
                    DCMotor.getFalcon500(1), // Drive motor is a Falcon 500
                    DCMotor.getFalcon500(1), // Steer motor is a Falcon 500
                    Constants.Swerve.FALCON_DRIVE_GEAR_RATIO, // Drive motor gear ratio.
                    Constants.Swerve.FALCON_TURN_GEAR_RATIO, // Steer motor gear ratio.
                    Volts.of(0.1),
                    Volts.of(0.2),
                    Inches.of(2),
                    KilogramSquareMeters.of(0.03),
                    COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof // Use the COF for Neoprene Tread
                    ))
            // Configures the track length and track width (spacing between swerve modules)
            .withTrackLengthTrackWidth(
                    Meters.of(Constants.Frame.ROBOT_WHEEL_DISTANCE_LENGTH),
                    Meters.of(Constants.Frame.ROBOT_WHEEL_DISTANCE_WIDTH))
            // Configures the bumper size (dimensions of the robot bumper)
            .withBumperSize(
                    Meters.of(Constants.Frame.FRAME_WITH_BUMPER_WIDTH),
                    Meters.of(Constants.Frame.FRAME_WITH_BUMPER_WIDTH))
            .withRobotMass(Kilograms.of(Constants.Frame.ROBOT_MASS));

    private final SwerveDriveSimulation swerveDriveSimulation;

    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    private int lastModifiersAppliedCount = 0;
    private SwerveModuleState[] lastModuleSetpoints = new SwerveModuleState[0];

    public Drivetrain() throws InvalidConfigException {
        ModuleConstants frontLeftConstants = Constants.Swerve.getFrontLeftConstants();
        ModuleConstants frontRightConstants = Constants.Swerve.getFrontRightConstants();
        ModuleConstants backLeftConstants = Constants.Swerve.getBackLeftConstants();
        ModuleConstants backRightConstants = Constants.Swerve.getBackRightConstants();

        SwerveModuleIO frontLeftIO;
        SwerveModuleIO frontRightIO;
        SwerveModuleIO backLeftIO;
        SwerveModuleIO backRightIO;

        if (Constants.RobotState.getMode() == Mode.REAL) {
            swerveDriveSimulation = null;

            frontLeftIO = new SwerveModuleReal(Constants.Swerve.PERIODIC, frontLeftConstants);
            frontRightIO = new SwerveModuleReal(Constants.Swerve.PERIODIC, frontRightConstants);
            backLeftIO = new SwerveModuleReal(Constants.Swerve.PERIODIC, backLeftConstants);
            backRightIO = new SwerveModuleReal(Constants.Swerve.PERIODIC, backRightConstants);

            // delay to wait for encoder values to stabilize
            Timer.delay(ENCODER_STABILIZATION_TIME);
        } else {
            /* Create a swerve drive simulation */
            swerveDriveSimulation = new SwerveDriveSimulation(
                    // Specify Configuration
                    driveTrainSimulationConfig,
                    // Specify starting pose
                    DashboardUI.Overview.getStartingLocation().getPose());

            // Register the drivetrain simulation to the default simulation world
            SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);

            frontLeftIO = new SwerveModuleSim(swerveDriveSimulation.getModules()[FL], frontLeftConstants);
            frontRightIO = new SwerveModuleSim(swerveDriveSimulation.getModules()[FR], frontRightConstants);
            backLeftIO = new SwerveModuleSim(swerveDriveSimulation.getModules()[BL], backLeftConstants);
            backRightIO = new SwerveModuleSim(swerveDriveSimulation.getModules()[BR], backRightConstants);
        }

        frontLeft = new SwerveModule(frontLeftConstants, frontLeftIO);
        frontRight = new SwerveModule(frontRightConstants, frontRightIO);
        backLeft = new SwerveModule(backLeftConstants, backLeftIO);
        backRight = new SwerveModule(backRightConstants, backRightIO);

        setpointGenerator = new SwerveSetpointGenerator(
                Constants.Swerve.PP_DEFAULT_CONFIG,
                Units.rotationsToRadians(
                        Constants.Swerve.TURN_MAX_ANGULAR_VELOCITY) // The max rotation velocity of a swerve module in
                // radians per second
                );

        // Initialize the previous setpoint to the robot's current speeds & module states
        ChassisSpeeds currentSpeeds = getChassisSpeeds();
        SwerveModuleState[] currentStates = getModuleStates();
        previousSetpoint = new SwerveSetpoint(
                currentSpeeds, currentStates, DriveFeedforwards.zeros(Constants.Swerve.PP_DEFAULT_CONFIG.numModules));

        sysIdRoutineDriveMotorsSpin = new SysIdRoutine(
                new SysIdRoutine.Config(
                        SYSID_DRIVE_RAMP_RATE, SYSID_DRIVE_STEP_VOLTAGE, SYSID_DRIVE_TIMEOUT, state -> {}),
                new SysIdRoutine.Mechanism(this::sysIdOnlyDriveMotorsSpin, null, this));

        sysIdRoutineDriveMotorsForward = new SysIdRoutine(
                new SysIdRoutine.Config(
                        SYSID_DRIVE_RAMP_RATE, SYSID_DRIVE_STEP_VOLTAGE, SYSID_DRIVE_TIMEOUT, state -> {}),
                new SysIdRoutine.Mechanism(this::sysIdOnlyDriveMotorsForward, null, this));

        sysIdRoutineTurnMotors = new SysIdRoutine(
                new SysIdRoutine.Config(SYSID_TURN_RAMP_RATE, SYSID_TURN_STEP_VOLTAGE, SYSID_TURN_TIMEOUT, state -> {}),
                new SysIdRoutine.Mechanism(this::sysIdOnlyTurnMotors, null, this));
    }

    public SwerveModule getFrontLeftModule() {
        return frontLeft;
    }

    public SwerveModule getFrontRightModule() {
        return frontRight;
    }

    public SwerveModule getBackLeftModule() {
        return backLeft;
    }

    public SwerveModule getBackRightModule() {
        return backRight;
    }

    public SwerveDriveSimulation getSwerveDriveSimulation() {
        return swerveDriveSimulation;
    }

    private static DrivetrainControl getDrivetrainControl() {
        if (RobotState.isTeleop()) {
            return DashboardUI.Overview.getControl().getDrivetrainControl();
        } else {
            return DrivetrainControl.createRobotRelative(Transform2d.kZero, Transform2d.kZero, Transform2d.kZero);
        }
    }

    /** Drives the robot using robot relative ChassisSpeeds. */
    private void driveInternal() {
        DrivetrainControl drivetrainControl = getDrivetrainControl();

        ChassisSpeeds nonDiscreteSpeeds = drivetrainControl.toChassisSpeeds(); // Converts the control to ChassisSpeeds

        // Note: it is important to not discretize speeds before or after
        // using the setpoint generator, as it will discretize them for you
        previousSetpoint = setpointGenerator.generateSetpoint(
                previousSetpoint, // The previous setpoint
                nonDiscreteSpeeds, // The desired target speeds
                RobotContainer.ROBOT_PERIODIC // The loop time of the robot code, in seconds
                );

        SwerveModuleState[] swerveModuleStates = previousSetpoint.moduleStates();

        // Sets state for each module
        if (SysIdManager.getSysIdRoutine() != SysIdManager.SysIdRoutine.DRIVETRAIN_SPIN
                && SysIdManager.getSysIdRoutine() != SysIdManager.SysIdRoutine.DRIVETRAIN_FORWARD) {
            frontLeft.setDesiredState(swerveModuleStates[FL]);
            frontRight.setDesiredState(swerveModuleStates[FR]);
            backLeft.setDesiredState(swerveModuleStates[BL]);
            backRight.setDesiredState(swerveModuleStates[BR]);
        }

        lastModuleSetpoints = swerveModuleStates;
    }

    @Override
    public void periodic() {
        driveInternal();

        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();
    }

    @Override
    public void simulationPeriodic() {
        frontLeft.simulationPeriodic();
        frontRight.simulationPeriodic();
        backLeft.simulationPeriodic();
        backRight.simulationPeriodic();
    }

    public void sysIdOnlyDriveMotorsSpin(Voltage volts) {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(360 - 45.0)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45.0)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(180 + 45.0)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(180 - 45.0)));

        frontLeft.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
        frontRight.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
        backLeft.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
        backRight.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
    }

    public void sysIdOnlyDriveMotorsForward(Voltage volts) {

        SwerveModuleState state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

        frontLeft.setDesiredState(state);
        frontRight.setDesiredState(state);
        backLeft.setDesiredState(state);
        backRight.setDesiredState(state);

        frontLeft.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
        frontRight.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
        backLeft.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
        backRight.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
    }

    public double sysIdOnlyGetDriveMotorVolts() {
        return SimpleMath.average4(
                frontLeft.getDriveMotorVoltsSysIdOnly(),
                frontRight.getDriveMotorVoltsSysIdOnly(),
                backLeft.getDriveMotorVoltsSysIdOnly(),
                backRight.getDriveMotorVoltsSysIdOnly());
    }

    public double sysIdOnlyGetDriveMotorPosition() {
        return SimpleMath.average4(
                frontLeft.getDriveWheelDistance(),
                frontRight.getDriveWheelDistance(),
                backLeft.getDriveWheelDistance(),
                backRight.getDriveWheelDistance());
    }

    public double sysIdOnlyGetDriveMotorVelocity() {
        return SimpleMath.average4(
                frontLeft.getDriveWheelVelocity(),
                frontRight.getDriveWheelVelocity(),
                backLeft.getDriveWheelVelocity(),
                backRight.getDriveWheelVelocity());
    }

    public void sysIdOnlyTurnMotors(Voltage volts) {
        frontLeft.setTurnMotorVoltsSysIdOnly(volts.in(Volts));
        frontRight.setTurnMotorVoltsSysIdOnly(volts.in(Volts));
        backLeft.setTurnMotorVoltsSysIdOnly(volts.in(Volts));
        backRight.setTurnMotorVoltsSysIdOnly(volts.in(Volts));
    }

    public double sysIdOnlyGetTurnMotorVolts() {
        return SimpleMath.average4(
                frontLeft.getTurnMotorVoltsSysIdOnly(),
                frontRight.getTurnMotorVoltsSysIdOnly(),
                backLeft.getTurnMotorVoltsSysIdOnly(),
                backRight.getTurnMotorVoltsSysIdOnly());
    }

    public double sysIdOnlyGetTurnMotorPosition() {
        return SimpleMath.average4(
                frontLeft.getTurnWheelRotation2d().getRotations(),
                frontRight.getTurnWheelRotation2d().getRotations(),
                backLeft.getTurnWheelRotation2d().getRotations(),
                backRight.getTurnWheelRotation2d().getRotations());
    }

    public double sysIdOnlyGetTurnMotorVelocity() {
        return SimpleMath.average4(
                frontLeft.getTurnWheelVelocity(),
                frontRight.getTurnWheelVelocity(),
                backLeft.getTurnWheelVelocity(),
                backRight.getTurnWheelVelocity());
    }

    /**
     * Retrieves the current chassis speeds relative to the robot's orientation.
     *
     * <p>This method calculates the chassis speeds based on the current states of all four swerve
     * modules using the drivetrain's kinematics.
     *
     * @return The current relative chassis speeds as a ChassisSpeeds object.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(
                frontLeft.getModuleState(),
                frontRight.getModuleState(),
                backLeft.getModuleState(),
                backRight.getModuleState());
    }

    /**
     * Retrieves the current chassis acceleration relative to the robot's orientation.
     *
     * <p>This method calculates the chassis acceleration based on the current states of all four
     * swerve modules using the drivetrain's kinematics.
     *
     * @return The current relative chassis acceleration as a ChassisSpeeds object.
     */
    public ChassisSpeeds getChassisAcceleration() {
        return kinematics.toChassisSpeeds(
                frontLeft.getModuleStateAcceleration(),
                frontRight.getModuleStateAcceleration(),
                backLeft.getModuleStateAcceleration(),
                backRight.getModuleStateAcceleration());
    }

    public int getModifiersAppliedCount() {
        return lastModifiersAppliedCount;
    }

    /**
     * Returns the swerve drive kinematics for this drivetrain.
     *
     * @return The SwerveDriveKinematics object associated with this drivetrain.
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getModulePosition(),
            frontRight.getModulePosition(),
            backLeft.getModulePosition(),
            backRight.getModulePosition()
        };
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getModuleState(),
            frontRight.getModuleState(),
            backLeft.getModuleState(),
            backRight.getModuleState()
        };
    }

    public SwerveModuleState[] getModuleSetpoints() {
        return lastModuleSetpoints;
    }

    public Command sysIdQuasistaticDriveMotorsSpin(SysIdRoutine.Direction direction) {
        return sysIdRoutineDriveMotorsSpin.quasistatic(direction);
    }

    public Command sysIdDynamicDriveMotorsSpin(SysIdRoutine.Direction direction) {
        return sysIdRoutineDriveMotorsSpin.dynamic(direction);
    }

    public Command sysIdQuasistaticDriveMotorsForward(SysIdRoutine.Direction direction) {
        return sysIdRoutineDriveMotorsForward.quasistatic(direction);
    }

    public Command sysIdDynamicDriveMotorsForward(SysIdRoutine.Direction direction) {
        return sysIdRoutineDriveMotorsForward.dynamic(direction);
    }

    public Command sysIdQuasistaticTurnMotors(SysIdRoutine.Direction direction) {
        return sysIdRoutineTurnMotors.quasistatic(direction);
    }

    public Command sysIdDynamicTurnMotors(SysIdRoutine.Direction direction) {
        return sysIdRoutineTurnMotors.dynamic(direction);
    }

    /** frees up all hardware allocations */
    @Override
    public void close() throws Exception {
        backLeft.close();
        backRight.close();
        frontLeft.close();
        frontRight.close();
    }
}
