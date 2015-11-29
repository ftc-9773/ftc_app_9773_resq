package org.robocracy.ftcrobot.DriverStation;

import com.qualcomm.ftcrobotcontroller.opmodes.DriveSystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Robocracy on 11/19/2015.
 */
public class DriverCommand {
    public enum HarvesterDirection {PULL, PUSH, NONE}
    public class DriveSystemCommand {
        // Vx = X-axis component of the desired Robot velocity.
        // Vy = Y-axis component of the desired Robot velocity
        // Omega = Desired angular velocity of the Robot
        // -1 <= Vx, Vy, Omega <= +1
        //  These values will be scaled to fit them into one of the 8 zones of the drive area
        public double angle, speedMultiplier, Omega;
    }

    public DriveSystemCommand drvsyscmd = new DriveSystemCommand();
    public class LinearLiftCommand {
        // Decide which button(s) on the gamepad(s) will be used for the lift.
        public float angle, direction;
    }
    public LinearLiftCommand linliftcmd = new LinearLiftCommand();

    public class HarvesterCommand {
        public HarvesterDirection direction;
    }
    public HarvesterCommand harvestercmd = new HarvesterCommand();

}
