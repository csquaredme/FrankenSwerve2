// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kButtonsPort = 1;
  }
  
  public static class SwerveConstants{
    public static final double kWheelRadius = 0.05845; //Need to update
    public static final int kAngleEncoderResolution = 7; //see https://www.andymark.com/products/hall-effect-two-channel-encoder
    public static final int kDriveEncoderResolution = 4096; // neo brushless settings in rev can id setting
  
    public static final double kModuleMaxAngularVelocity = DriveConstants.kMaxAngularSpeed;
    public static final double kModuleMaxAngularAcceleration =
        2 * Math.PI; // radians per second squared
        
    public static final double kWheelDiameterMeters = kWheelRadius * 2;
    public static final int kDrivingMotorPinionTeeth = 14;
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second    
    public static final int driveGainP = 1;
    public static final int driveGainI = 0;
    public static final int driveGainD = 0;
  }

  public static class DriveConstants {
    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
    public static final double kDriveDeadband = 0.05;
  }

  public static class ControlSystem {
    public static final int kLeftFrontDrive = 1;
    public static final int kLeftBackDrive = 2;
    public static final int kRightFrontDrive = 3;
    public static final int kRightBackDrive = 4;
    public static final int kLeftFrontTurn = 5;
    public static final int kLeftBackTurn = 6;
    public static final int kRightFrontTurn = 7;
    public static final int kRightBackTurn = 8;

    public static final int kLFturnA = 0;
    public static final int kLFturnB = 1;
    public static final int kLBturnA = 4;
    public static final int kLBturnB = 5;
    public static final int kRFturnA = 2;
    public static final int kRFturnB = 3;
    public static final int kRBturnA = 6;
    public static final int kRBturnB = 7;
  }

  public static class AutoConstants {}
}
