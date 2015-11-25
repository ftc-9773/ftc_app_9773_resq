package org.robocracy.ftcrobot.DriverStation;

import com.qualcomm.ftcrobotcontroller.opmodes.DriveSystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Robocracy on 11/19/2015.
 */
public class DriverCommand {
    public enum WinchDirection {PULL, PUSH}
    public enum LiftDirection {EXPAND, COLLAPSE}
    public class DriveSystemCommand {
        // Vx = X-axis component of the desired Robot velocity.
        // Vy = Y-axis component of the desired Robot velocity
        // Omega = Desired angular velocity of the Robot
        // -1 <= Vx, Vy, Omega <= +1
        //  These values will be scaled to fit them into one of the 8 zones of the drive area
        double angle, speedMultiplier, Omega;
    }

    DriveSystemCommand drvsyscmd = new DriveSystemCommand();
    public class LinearLiftCommand {
        // Decide which button(s) on the gamepad(s) will be used for the lift.
        double power;
        LiftDirection direction;
    }

    public class WinchCommand {
        double power;
        WinchDirection direction;
    }

}
