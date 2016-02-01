package org.robocracy.ftcrobot;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.Servo;

import org.robocracy.ftcrobot.DriverStation.DriverStation;
import org.robocracy.ftcrobot.FTCRobot;
import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

/**
 * Operates Climber release servos on robot
 * @author Team Robocracy
 */
public class RightClimber {
    FTCRobot robot;
    LinearOpMode curOpMode;
    Servo rightClimber;
    boolean rightClimberAvailable = false;

    public RightClimber(FTCRobot robot, Servo rightClimber, LinearOpMode curOpMode, boolean allianceIsBlue){
        this.robot = robot;
        this.curOpMode = curOpMode;
        if (rightClimber != null) {
            rightClimberAvailable = true;
            this.rightClimber = rightClimber;
            this.rightClimber.scaleRange(0.1, 0.47);
            this.rightClimber.setPosition(1.0); // 1.0 == 0.47 because of scaleRange() call above
            DbgLog.msg(String.format("RightClimber Position = %f", this.rightClimber.getPosition()));
            if(allianceIsBlue){
                this.rightClimber.close();
            }
        }
    }

    /**
     * Moves right climber servo based on {@code enum rightClimberDirection} value set in {@link DriverStation#getNextClimberCmd()}
     * @param drvrcmd {@link DriverCommand} object with values.
     */
    public void applyDSCmd(DriverCommand drvrcmd){
        if (!rightClimberAvailable) {
            return;
        }
        double rightClimberPosition = rightClimber.getPosition();
        DbgLog.msg(String.format("RightClimber Position = %f", rightClimberPosition));
        switch (drvrcmd.rightClimberCmd.rightClimberStatus){
            case -1:
                rightClimber.setPosition(Range.clip(0, 0, 1));
                break;
            case 1:
                rightClimber.setPosition(Range.clip(1, 0, 1));
                break;
            case 0:
                break;
            default:
                break;
        }
    }
}
