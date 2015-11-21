package org.robocracy.ftcrobot.DriveSystem;

/**
 * Created by burugula on 11/16/2015.
 */
enum WheelType  {Mecanum, Omni, Tetrix};

public abstract class Wheel {
    double frictionCoeffMatStraight;
    double frictionCoeffMatStrafe;
//    double frictionCoefficientMountain;
    double diameter, circumference;
    WheelType wheelType;

        public Wheel(WheelType wheelType, double diameter, double coeffMatStraight, double coeffMatStrafe) {
            this.wheelType = wheelType;
            this.diameter = diameter;
            this.circumference = Math.PI * diameter;
            this.frictionCoeffMatStraight = coeffMatStraight;
            this.frictionCoeffMatStrafe = coeffMatStrafe;
        }
}
