package frc.robot.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.FieldStartingLocation;
import frc.robot.control.AbstractControl;
import frc.robot.utils.libraries.Elastic;
import java.util.function.Supplier;

public final class OverviewLayout extends AbstractLayout {

    public enum DriverOrientation {
        X_AXIS_TOWARDS_TRIGGER("Competition"),
        Y_AXIS_TOWARDS_TRIGGER("Y Axis"),
        X_AXIS_INVERTED_TOWARDS_TRIGGER("Couch Drive");

        public final String displayName;

        DriverOrientation(String orientation) {
            displayName = orientation;
        }
    }

    private final Field2d field = new Field2d();

    private AbstractControl defaultControl;
    private AbstractControl testControl;

    private Supplier<Boolean> navSensorValue = () -> false;

    public OverviewLayout() {
        buildSendable("Field", field);
        addValueSendable("Nav Sensor", () -> navSensorValue.get(), "boolean");

        SmartDashboard.putBoolean("Overview/ResetLocationButton", false);
        SmartDashboard.putBoolean("Overview/EncoderReset", false);
    }

    /**
     * Initializes the control object
     *
     * @param defaultControl the first term will always be the default control object
     * @param controls any other control objects you want to initialize
     */
    public void addControls(AbstractControl defaultControl, AbstractControl... controls) {
        this.defaultControl = defaultControl;
    }

    public void setNavSensor(Supplier<Boolean> navSensor) {
        navSensorValue = navSensor;
    }

    @Override
    public void switchTo() {
        Elastic.selectTab("Overview");
    }

    @Override
    protected NetworkTable getNetworkTable() {
        return NetworkTableInstance.getDefault().getTable("/SmartDashboard/Overview");
    }

    public DriverOrientation getDriverOrientation() {
        return DriverOrientation.X_AXIS_INVERTED_TOWARDS_TRIGGER;
    }

    public void setTestControl(AbstractControl testControl) {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.TEST) return;
        this.testControl = testControl;
    }

    public AbstractControl getControl() {
        if (Constants.RobotState.getMode() == Constants.RobotState.Mode.TEST) {
            if (testControl == null) {
                throw new IllegalStateException("Test control is not set!");
            }
            return testControl;
        }

        return defaultControl;
    }

    public void setRobotPose(Pose2d pose) {
        field.setRobotPose(pose);
    }

    public void setVisionPose(String name, Pose2d pose) {
        field.getObject(name).setPose(pose);
    }

    public FieldStartingLocation getStartingLocation() {
        return FieldStartingLocation.DEFAULT;
    }

    public boolean isResetLocationPressed() {
        return SmartDashboard.getBoolean("Overview/ResetLocationButton", false);
    }

    public boolean isEncoderResetPressed() {
        return SmartDashboard.getBoolean("Overview/EncoderReset", false);
    }
}
