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
        //armLength = extending/collapsing component of Linear Lift
        public float angle, armLength;
    }
    public LinearLiftCommand linliftcmd = new LinearLiftCommand();

    public enum HarvesterDirection {NONE, PULL, PUSH}
    public class HarvesterCommand {
        public HarvesterDirection direction;
    }
    public HarvesterCommand harvestercmd = new HarvesterCommand();

    public class LatchCommand{
        //Convention: 0 = none, 1 = up, -1 = down
        public int latchStatus;
    }
    public LatchCommand latchCmd = new LatchCommand();

    public enum BucketDirection {NONE, LEFT, RIGHT}
    public class BucketCommand{
        public BucketDirection direction;
    }
    public BucketCommand bucketCmd = new BucketCommand();

    public class LeftClimberCommand{
        //Convention: 0 = none, 1 = up, -1 = down
        public int leftClimberStatus;
    }
    public LeftClimberCommand leftClimberCmd = new LeftClimberCommand();

    public class RightClimberCommand{
        //Convention: 0 = none, 1 = up, -1 = down
        public int rightClimberStatus;
    }
    public RightClimberCommand rightClimberCmd = new RightClimberCommand();

    public long timeStamp; // timestamp of the driver command; this is used for autonomous reply
    public class SensorValues{
        double yaw, pitch, ods, colorRed, colorGreen, colorBlue;
    }
    public SensorValues sensorValues = new SensorValues();
    public enum EndGameStatus{NONE, RUN, STOP}
    public class RunEndGame{
        public  EndGameStatus endGameStatus;
    }
    public RunEndGame runEndGame = new RunEndGame();

    public class ClimberDispenserCommand{
        //Convention: 0 = none, 1 = up, -1 = down
        public int climberDispenserStatus;
    }
    public ClimberDispenserCommand climberDispenserCommand = new ClimberDispenserCommand();
}