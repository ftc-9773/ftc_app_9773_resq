package org.robocracy.ftcrobot.DriveSystem;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by pranavb on 11/8/15.
 */


public abstract interface DriveSystemInterface {
    public static enum RobotDirection {FORWARD, BACKWARD, LEFT, RIGHT};
    final static int ENCODER_CPR = 1120;
    final static double GEAR_RATIO = 1;
    final static double diameter = 4.0;

    public abstract void autoMove(RobotDirection direction, double distance, double speed)  throws InterruptedException;

    public abstract void autoMove(RobotDirection direction, int intensity, double speed, OpticalDistanceSensor ods) throws InterruptedException;

    public abstract void autoTurn(RobotDirection direction, double degrees, double speed)  throws InterruptedException;

    public abstract void driverMove(RobotDirection dir, double speed)  throws InterruptedException;

    public abstract void driverTurn(RobotDirection direction, double speed)  throws InterruptedException;

    // This will have to be removed later.
    public abstract void mecanumWheelDrive(float strafeDirection, float strafeThrottle, float turnDirection, float turnThrottle);

}
