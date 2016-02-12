package org.robocracy.ftcrobot;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.robocracy.ftcrobot.DriverStation.DriverCommand;
import org.robocracy.ftcrobot.DriverStation.DriverStation;

/**
 * Operates climber dispenser on robot
 * @author Team Robocracy
 */
public class ClimberDispenser {
    FTCRobot robot;
    LinearOpMode curOpMode;
    Servo climberDispenserServo;
    double climberDispenserServoPosition;
    boolean climberDispenserServoAvailable = false;

    public ClimberDispenser(FTCRobot robot, Servo climberDispenserServo, LinearOpMode curOpMode){
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.climberDispenserServo = climberDispenserServo;
        if (climberDispenserServo != null) {
            DbgLog.msg(String.format("climber dispenser position = %f", this.climberDispenserServo.getPosition()));
            this.climberDispenserServo.scaleRange(0.157, 0.784);
            this.climberDispenserServo.setDirection(Servo.Direction.REVERSE);
            this.climberDispenserServo.setPosition(0);
            climberDispenserServoPosition = 0.0;
            this.climberDispenserServoAvailable = true;
        }
    }

    /**
     * Moves climber dispenser servo based on {@code enum climerDispenserStatus} value set in {@link DriverStation#getNextClimberCmd()}
     * @param drvrcmd {@link DriverCommand} object with values.
     */
    public void applyDSCmd(DriverCommand drvrcmd){
        if (!climberDispenserServoAvailable) {
            return;
        }
        DbgLog.msg(String.format("applyDSCmd: climberDispenserServoPosition variable = %f", climberDispenserServoPosition));
        DbgLog.msg(String.format("applyDSCmd: ClimberDispenser.sevo.getPosition = %f", climberDispenserServo.getPosition()));

        switch (drvrcmd.climberDispenserCommand.climberDispenserStatus){
            case -1:
                climberDispenserServoPosition = Range.clip(climberDispenserServoPosition+0.02, 0, 1);
                climberDispenserServo.setPosition(climberDispenserServoPosition);
                break;
            case 1:
                climberDispenserServoPosition = Range.clip(climberDispenserServoPosition-0.02, 0, 1);
                DbgLog.msg(String.format("Setting climberDispenserposition to %f", Range.clip(climberDispenserServoPosition-0.1, 0, 1)));
                climberDispenserServo.setPosition(climberDispenserServoPosition);
                break;
            case 0:
                break;
            default:
                break;
        }
    }
}
