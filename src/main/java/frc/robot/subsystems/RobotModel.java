package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Represents the physical model of the robot, including mechanisms and their positions */
public final class RobotModel extends SubsystemBase {

    public RobotModel() {
        periodic();
    }

    @Override
    public void periodic() {}

    @SuppressWarnings("java:S2325") // rest of the getters are non-static
    public Pose2d getRobot() {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            return RobotContainer.drivetrain.getSwerveDriveSimulation().getSimulatedDriveTrainPose();
        } else {
            return Pose2d.kZero;
        }
    }
}
