package com.qualcomm.ftcrobotcontroller.opmodes;

/*import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;*/
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.ftcrobotcontroller.opmodes.DriveSystem;

/**
 * Created by pranavb on 11/11/15.
 */
public class Autonomous extends /*LinearOpMode*/ OpMode {

    @Override
    public void init(){
        DcMotor frontLeft = hardwareMap.dcMotor.get("fMotorL");
        DcMotor frontRight = hardwareMap.dcMotor.get("fMotorR");
        DcMotor rearLeft = hardwareMap.dcMotor.get("rMotorL");
        DcMotor rearRight = hardwareMap.dcMotor.get("rMotorR");
        /*rearLeft.setDirection(DcMotor.Direction.REVERSE);*/
        rearRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        /*frontRight.setDirection(DcMotor.Direction.REVERSE);*/
        DriveSystem mecanumDriveSystem = new DriveSystem(frontLeft, frontRight, rearLeft, rearRight);

        /*waitOneFullHardwareCycle();
        waitForStart();*/
        double counts = mecanumDriveSystem.mecanumWheelAutoDrive(60, 0.5);

        /*while(opModeIsActive()){


            telemetry.addData("Counts to move: ", counts);
        }*/
    }

    @Override
    public void loop(){

    }
}
