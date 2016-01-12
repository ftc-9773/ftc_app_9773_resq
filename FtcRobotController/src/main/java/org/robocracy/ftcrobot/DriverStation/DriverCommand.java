package org.robocracy.ftcrobot.DriverStation;

import com.qualcomm.ftcrobotcontroller.opmodes.DriveSystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.robocracy.ftcrobot.util.FileRW;

import org.robocracy.ftcrobot.Latch;

/**
 * @author Team Robocracy
 *
 * Contains various {@code DriverCommand}s, for various robot systems.
 *
 * @see {@link DriverStation}
 */
public class DriverCommand {
    public enum HarvesterDirection {PULL, PUSH, NONE}
    public enum LatchDirection {DOWN, UP, NONE}
    public class DriveSystemCommand {
        // angle = X-axis component of the desired Robot velocity.
        // speedMultiplier = Y-axis component of the desired Robot velocity
        // Omega = Desired angular velocity of the Robot
        // -1 <= angle, speedMultiplier, Omega <= +1
        //  These values will be scaled to fit them into one of the 8 zones of the drive area
        public double angle, speedMultiplier, Omega;
    }

    public DriveSystemCommand drvsyscmd = new DriveSystemCommand();
    public class LinearLiftCommand {
        //angle = altitude component of Linear Lift
        //direction = extending/collapsing component of Linear Lift
        public float angle, direction;
    }
    public LinearLiftCommand linliftcmd = new LinearLiftCommand();

    public class HarvesterCommand {
        public HarvesterDirection direction;
    }
    public HarvesterCommand harvestercmd = new HarvesterCommand();

    public class LatchCommand{
        public LatchDirection direction;
    }
    public LatchCommand latchCmd = new LatchCommand();

    public enum BucketDirection {LEFT, RIGHT, NONE}
    public class BucketCommand{
        public BucketDirection direction;
    }
    public BucketCommand bucketCmd = new BucketCommand();

    public enum LeftClimberDirection {DOWN, UP, NONE}
    public class LeftClimberCommand{
        public LeftClimberDirection leftClimberDirection;
    }
    public LeftClimberCommand leftClimberCmd = new LeftClimberCommand();

    public enum RightClimberDirection {DOWN, UP, NONE}
    public class RightClimberCommand{
        public RightClimberDirection rightClimberDirection;
    }
    public RightClimberCommand rightClimberCmd = new RightClimberCommand();
}