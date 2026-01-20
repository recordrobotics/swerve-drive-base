// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.ModuleConstants;
import frc.robot.utils.ModuleConstants.DriveMotorType;
import frc.robot.utils.ModuleConstants.InvalidConfigException;
import frc.robot.utils.ModuleConstants.MotorLocation;
import frc.robot.utils.ModuleConstants.TurnMotorType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    private Constants() {}

    public enum FieldStartingLocation {
        DEFAULT(new Pose2d(7.145, 4.026, Rotation2d.fromDegrees(180.000)));

        private final Pose2d transformRed;
        private final Pose2d transformBlue;

        private FieldStartingLocation(Pose2d poseBlue) {
            transformRed = FlippingUtil.flipFieldPose(poseBlue);
            transformBlue = poseBlue;
        }

        public Pose2d getPose() {
            return DriverStationUtils.getCurrentAlliance() == Alliance.Red ? transformRed : transformBlue;
        }
    }

    public final class Control {

        // Sensitivity for speed meter
        public static final double DIRECTIONAL_SPEED_METER_LOW = 0.25;
        public static final double DIRECTIONAL_SPEED_METER_HIGH = 4.7;
        public static final double SPIN_SPEED_METER_LOW = 0.5;
        public static final double SPIN_SPEED_METER_HIGH = 2.4;

        // Sensitivies for directional controls (XY) and spin (theta)
        public static final double JOYSTICK_DIRECTIONAL_SENSITIVITY = 1;
        public static final double JOYSTICK_SPIN_SENSITIVITY = 2;
        public static final double JOYSTICK_X_THRESHOLD = 0.15;
        public static final double JOYSTICK_Y_THRESHOLD = 0.15;
        public static final double JOYSTICK_SPIN_THRESHOLD = 0.3;

        private Control() {}
    }

    public final class Frame {

        /**
         * Distance between wheels (width aka between left and right and length aka between front and
         * back). Used for calculating wheel locations on the robot
         */
        public static final double ROBOT_WHEEL_DISTANCE_WIDTH = 0.5588;

        public static final double ROBOT_WHEEL_DISTANCE_LENGTH = 0.5588;

        public static final double FRAME_WIDTH = Inches.of(30).in(Meters);
        public static final double FRAME_WITH_BUMPER_WIDTH =
                FRAME_WIDTH + Inches.of(6.5).in(Meters);
        public static final double MAX_MECHANISM_HEIGHT = 2.1336;

        public static final double ROBOT_MASS = 27.4; // kg
        public static final double ROBOT_MOI = 14.547; // kg*m^2

        private Frame() {}
    }

    public final class Swerve {

        public static final double PERIODIC = 0.02;

        // Works out module locations
        public static final double WHEEL_EXTENT_X = Frame.ROBOT_WHEEL_DISTANCE_WIDTH / 2;
        public static final double WHEEL_EXTENT_Y = Frame.ROBOT_WHEEL_DISTANCE_LENGTH / 2;
        // Distance from center of robot to wheel
        public static final double WHEEL_BASE_RADIUS = Math.hypot(WHEEL_EXTENT_X, WHEEL_EXTENT_Y);

        public static final Translation2d FRONT_LEFT_WHEEL_LOCATION = new Translation2d(WHEEL_EXTENT_X, WHEEL_EXTENT_Y);
        public static final Translation2d FRONT_RIGHT_WHEEL_LOCATION =
                new Translation2d(WHEEL_EXTENT_X, -WHEEL_EXTENT_Y);
        public static final Translation2d BACK_LEFT_WHEEL_LOCATION = new Translation2d(-WHEEL_EXTENT_X, WHEEL_EXTENT_Y);
        public static final Translation2d BACK_RIGHT_WHEEL_LOCATION =
                new Translation2d(-WHEEL_EXTENT_X, -WHEEL_EXTENT_Y);

        // Gear ratios for falcon and kraken
        public static final double FALCON_TURN_GEAR_RATIO =
                15.43; // (https://web.archive.org/web/20230117081053/https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options)
        public static final double FALCON_DRIVE_GEAR_RATIO =
                7.36; // (https://web.archive.org/web/20230117081053/https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options)

        public static final double KRAKEN_TURN_GEAR_RATIO = 13.3714;
        public static final double KRAKEN_DRIVE_GEAR_RATIO = 6.75; // X1 12 pinion

        public static final Current FALCON_TURN_STATOR_CURRENT_LIMIT = Amps.of(100);
        public static final Current FALCON_TURN_SUPPLY_CURRENT_LIMIT = Amps.of(25);
        public static final Current FALCON_DRIVE_STATOR_CURRENT_LIMIT = Amps.of(120);
        public static final Current FALCON_DRIVE_SUPPLY_CURRENT_LIMIT = Amps.of(52);

        public static final Current KRAKEN_TURN_STATOR_CURRENT_LIMIT = Amps.of(100);
        public static final Current KRAKEN_TURN_SUPPLY_CURRENT_LIMIT = Amps.of(25);
        public static final Current KRAKEN_DRIVE_STATOR_CURRENT_LIMIT = Amps.of(120);
        public static final Current KRAKEN_DRIVE_SUPPLY_CURRENT_LIMIT = Amps.of(52);

        public static final double FALCON_DRIVE_KS = 0.13192;
        public static final double FALCON_DRIVE_KV = 2.7547;
        public static final double FALCON_DRIVE_KA = 0.24758;
        public static final double FALCON_DRIVE_KP = 4.6957;

        public static final double FALCON_TURN_KV = 1.7214;
        public static final double FALCON_TURN_KA = 0.049311;
        public static final double FALCON_TURN_KS = 0.13609;
        public static final double FALCON_TURN_KP = 67.02;
        public static final double FALCON_TURN_KD = 3.2831;

        public static final double KRAKEN_DRIVE_KS = 0.13192;
        public static final double KRAKEN_DRIVE_KV = 2.7547;
        public static final double KRAKEN_DRIVE_KA = 0.24758;
        public static final double KRAKEN_DRIVE_KP = 4.6957;

        public static final double KRAKEN_TURN_KV = 1.2993;
        public static final double KRAKEN_TURN_KA = 0.058972;
        public static final double KRAKEN_TURN_KS = 0.51562;
        public static final double KRAKEN_TURN_KP = 55.543;
        public static final double KRAKEN_TURN_KD = 2.3952;

        // Wheel diameter
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

        // Turn max velocity and acceleration
        // Calculated from motor rpm 5000 / 60 (rps) / gear ratio (15.43)
        public static final double TURN_MAX_ANGULAR_VELOCITY = 5; // ROTATIONS / SECOND
        // Calculated from max velocity / time to reach (0.1)
        public static final double TURN_MAX_ANGULAR_ACCELERATION = 20; // ROTATIONS / SECOND / SECOND

        public static final double DRIVE_MAX_ACCELERATION = 13.18;
        public static final double DRIVE_MAX_JERK = 131.28;

        public static final double TURN_MAX_JERK = 1600; // TODO: tune
        public static final double TURN_MMEXPO_KV = 5.8; // TODO: tune
        public static final double TURN_MMEXPO_KA = 1.5; // TODO: tune

        /** The max speed the robot can travel safely */
        public static final double ROBOT_MAX_SPEED = 4.35;

        public static final RobotConfig PP_DEFAULT_CONFIG = new RobotConfig(
                Constants.Frame.ROBOT_MASS,
                Constants.Frame.ROBOT_MOI,
                new ModuleConfig(
                        WHEEL_DIAMETER / 2,
                        ROBOT_MAX_SPEED,
                        1.2,
                        DCMotor.getFalcon500(1),
                        Constants.Swerve.FALCON_DRIVE_GEAR_RATIO,
                        52,
                        1),
                FRONT_LEFT_WHEEL_LOCATION,
                FRONT_RIGHT_WHEEL_LOCATION,
                BACK_LEFT_WHEEL_LOCATION,
                BACK_RIGHT_WHEEL_LOCATION);

        public static final PPHolonomicDriveController PP_DRIVE_CONTROLLER = new PPHolonomicDriveController(
                new PIDConstants(1.5, 0.0, 0.0), // Translation PID constants
                new PIDConstants(1.5, 0.0, 0.0) // Rotation PID constants
                );

        private Swerve() {}

        // Module Creation

        public static ModuleConstants getFrontLeftConstants() throws InvalidConfigException {
            return ModuleConstants.fromConfig(MotorLocation.FRONT_LEFT, DriveMotorType.FALCON, TurnMotorType.FALCON);
        }

        public static ModuleConstants getFrontRightConstants() throws InvalidConfigException {
            return ModuleConstants.fromConfig(MotorLocation.FRONT_RIGHT, DriveMotorType.FALCON, TurnMotorType.FALCON);
        }

        public static ModuleConstants getBackLeftConstants() throws InvalidConfigException {
            return ModuleConstants.fromConfig(MotorLocation.BACK_LEFT, DriveMotorType.FALCON, TurnMotorType.FALCON);
        }

        public static ModuleConstants getBackRightConstants() throws InvalidConfigException {
            return ModuleConstants.fromConfig(MotorLocation.BACK_RIGHT, DriveMotorType.FALCON, TurnMotorType.FALCON);
        }
    }

    public final class RobotState {

        public static final boolean MOTOR_LOGGING_ENABLED = false;

        /**
         * <p>
         * Enable NT and Advantage Scope for unit tests.
         * </p>
         * <p>
         * WARNING! ONLY ENABLE THIS IF RUNNING A SINGLE UNIT TEST
         * </p>
         * <p>
         * RUNNING TESTS IN PARALLEL IS NOT SUPPORTED
         * </p>
         *
         * Example: {@code ./gradlew test --tests "*ReefAutoScoreTestCases`$Blue4"}
         */
        public static final boolean UNIT_TESTS_ENABLE_ADVANTAGE_SCOPE = false;

        private static boolean runningAsUnitTest = false;

        private RobotState() {}

        public static void setRunningAsUnitTest() {
            runningAsUnitTest = true;
        }

        public static Mode getMode() {
            if (RobotBase.isReal()) return Mode.REAL;
            if (runningAsUnitTest) return Mode.TEST;
            return RobotBase.isSimulation() ? Mode.SIM : Mode.REPLAY;
        }

        public enum Mode {
            REAL(true),
            SIM(true),
            REPLAY(false),
            TEST(false);

            boolean realtime;

            Mode(boolean realtime) {
                this.realtime = realtime;
            }

            public boolean isRealtime() {
                return realtime;
            }
        }
    }
}
