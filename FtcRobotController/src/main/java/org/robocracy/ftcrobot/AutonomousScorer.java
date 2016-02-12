package org.robocracy.ftcrobot;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.robocracy.ftcrobot.DriveSystem.AWDMecanumDS;
import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import org.robocracy.ftcrobot.util.FileRW;

import java.util.concurrent.TimeUnit;

/**
 * Runs robot during Autonomous mode.
 * @author Team Robocracy
 */
public class AutonomousScorer {
    FTCRobot robot;
    LinearOpMode curOpMode;
    boolean  allianceIsBlue;

    public OpticalDistanceSensor ods;
    public AutonomousScorer(FTCRobot robot, LinearOpMode curOpMode, boolean allianceIsBlue) {
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.allianceIsBlue = allianceIsBlue;
    }

    /**
     * Drives robot during Autonomous based on values recorded in .csv file at {@code filepath}.
     * @throws InterruptedException
     */
    public void driveUsingReplay() throws InterruptedException {
        DriverCommand drvrCmd;
        long replayStartTime;
        FileRW readFileRW;
        readFileRW = this.robot.readFileRW;

        String line = readFileRW.getNextLine();
        // Note the starting timestamp
        replayStartTime = System.nanoTime();
        while (line != null){
            this.curOpMode.waitForNextHardwareCycle();
            drvrCmd = robot.drvrStation.getNextCommand(line);
            if ((System.nanoTime() - replayStartTime) < drvrCmd.timeStamp) {
                // Wait for a few nano seconds
                // This will not be precise but that should be okay
                TimeUnit.NANOSECONDS.sleep(drvrCmd.timeStamp - ((System.nanoTime() - replayStartTime)));
            }
            robot.driveSys.applyCmd(drvrCmd);
            robot.linearLift.applyCmd(drvrCmd);
            drvrCmd.harvestercmd.direction = DriverCommand.HarvesterDirection.PUSH;
            robot.harvester.applyDSCmd(drvrCmd);
            drvrCmd.latchCmd.latchStatus = -1;
            robot.latch.applyDSCmd(drvrCmd);
            robot.climberDispenser.applyDSCmd(drvrCmd);
            line = readFileRW.getNextLine();
        }
        robot.driveSys.stopDriveSystem();
        this.curOpMode.waitForNextHardwareCycle();
    }

    /**
     * Drives robot during End Game based on values recorded in .csv file
     * @param isEndGame parameter that, when provided, overrides {@link #driveUsingReplay()}.
     * @throws InterruptedException
     */
    public void driveUsingReplay(boolean isEndGame) throws InterruptedException{
        if(isEndGame) {
            DriverCommand drvrCmd;
            long replayStartTime;
            FileRW readFileRW;
            readFileRW = this.robot.readFileRW;

            String line = readFileRW.getNextLine();
            // Note the starting timestamp
            replayStartTime = System.nanoTime();
            while (line != null) {
                this.curOpMode.waitForNextHardwareCycle();
                drvrCmd = robot.drvrStation.getNextCommand(line, true);
//                DbgLog.msg(String.format("line = %s", line));
                if ((System.nanoTime() - replayStartTime) < drvrCmd.timeStamp) {
                    // Wait for a few nano seconds
                    // This will not be precise but that should be okay
                    TimeUnit.NANOSECONDS.sleep(drvrCmd.timeStamp - ((System.nanoTime() - replayStartTime)));
                }
                robot.driveSys.applyCmd(drvrCmd);
                robot.linearLift.applyCmd(drvrCmd);
                robot.latch.applyDSCmd(drvrCmd);
                robot.rightClimber.applyDSCmd(drvrCmd);
                robot.leftClimber.applyDSCmd(drvrCmd);
                line = readFileRW.getNextLine();
            }
            robot.driveSys.stopDriveSystem();
            this.curOpMode.waitForNextHardwareCycle();
        }
    }

    public void strafeTheDistance(AWDMecanumDS drivesys, int distanceToStrafe) throws InterruptedException {
        int angle, speed;
        if (distanceToStrafe < 0) {
            angle = 180;
        }
        else {
            angle = 0;
        }
        speed = 12;
        drivesys.autoMecanum(angle, distanceToStrafe, speed, 0);
    }
}