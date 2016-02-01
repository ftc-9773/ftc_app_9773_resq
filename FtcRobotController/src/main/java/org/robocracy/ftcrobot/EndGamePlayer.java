package org.robocracy.ftcrobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import org.robocracy.ftcrobot.util.FileRW;

import java.io.IOException;

/**
 * Created by pranavb on 1/24/16.
 */

/**
 * Replays pre-recoded end game
 * @author Team Robocracy
 */
public class EndGamePlayer {
    FTCRobot robot;
    AutonomousScorer autoScorer;
    LinearOpMode curOpMode;
    String filePath;
    FileRW fileRW;

    public EndGamePlayer(FTCRobot robot, LinearOpMode curOpMode, boolean allianceIsBlue){
        this.robot = robot;
        this.curOpMode = curOpMode;
        if(allianceIsBlue){
            filePath = "/sdcard/FIRST/autonomousCmds/endGameBlue.csv";
        }
        else{
            filePath = "/sdcard/FIRST/autonomousCmds/endGameRed.csv";
        }
        this.fileRW = new FileRW(filePath, false);
        this.autoScorer = new AutonomousScorer(robot, curOpMode, allianceIsBlue);
    }

    public void runEndGame(DriverCommand drvrCmd) throws InterruptedException{
        if(drvrCmd.runEndGame.endGameStatus == DriverCommand.EndGameStatus.RUN){
            autoScorer.driveUsingReplay();
            try {
                if (fileRW != null){
                    fileRW.close();
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
