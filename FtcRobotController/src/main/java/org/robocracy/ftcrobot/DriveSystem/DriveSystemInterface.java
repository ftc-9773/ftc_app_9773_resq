package org.robocracy.ftcrobot.DriveSystem;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by pranavb on 11/8/15.
 */


public interface DriveSystemInterface {
    static enum RobotDirection {FORWARD, BACKWARD, LEFT, RIGHT};
    final int ENCODER_CPR = 1120;
    final double GEAR_RATIO = 1;
    final double diameter = 4.0;

    void autoMove(RobotDirection direction, double distance, double speed)  throws InterruptedException;

    void autoMove(RobotDirection direction, int intensity, double speed, OpticalDistanceSensor ods) throws InterruptedException;

    void autoTurn(RobotDirection direction, double degrees, double speed)  throws InterruptedException;

    // angle is specified in degrees:  -360 < angle < 360
    void autoMecanum(int angle, int distance, double speed, int robotSpinDegrees) throws InterruptedException;

    void driveMecanum(int angle, double speedMultiplier, double Omega) throws  InterruptedException;

    void driverMove(RobotDirection dir, double speed)  throws InterruptedException;

    void driverTurn(RobotDirection direction, double speed)  throws InterruptedException;

    // This will have to be removed later.
    void mecanumWheelDrive(float strafeDirection, float strafeThrottle, float turnDirection, float turnThrottle);

}
