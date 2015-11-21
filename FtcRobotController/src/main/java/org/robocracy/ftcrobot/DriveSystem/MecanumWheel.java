package org.robocracy.ftcrobot.DriveSystem;

/**
 * Created by burugula on 11/16/2015.
 */
public class MecanumWheel extends Wheel {
    public MecanumWheel(double diameter, double frictionCoeffMatStraight, double frictionCoeffMatStrafe){
        super(WheelType.Mecanum, diameter, frictionCoeffMatStraight, frictionCoeffMatStrafe);
    }
}
