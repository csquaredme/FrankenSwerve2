// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ControlSystem;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;


public class DriveTrain extends SubsystemBase {
  
  private final Translation2d m_frontLeftLocation = new Translation2d(0.362, 0.235);
  private final Translation2d m_frontRightLocation = new Translation2d(0.362, -0.235);
  private final Translation2d m_backLeftLocation = new Translation2d(0.362, -0.235);
  private final Translation2d m_backRightLocation = new Translation2d(0.362, 0.235);

  private final SwerveModule m_frontLeft= new SwerveModule(
          ControlSystem.kLeftFrontDrive,
          ControlSystem.kLeftFrontTurn, 
          ControlSystem.kLFturnA, 
          ControlSystem.kLFturnB,
          DriveConstants.kFrontLeftChassisAngularOffset);
  private final SwerveModule m_frontRight = new SwerveModule(
          ControlSystem.kRightFrontDrive,
          ControlSystem.kRightFrontTurn, 
          ControlSystem.kRFturnA, 
          ControlSystem.kRFturnB,
          DriveConstants.kFrontRightChassisAngularOffset);
  private final SwerveModule m_backLeft = new SwerveModule(
          ControlSystem.kLeftBackDrive,
          ControlSystem.kLeftBackTurn, 
          ControlSystem.kLBturnA, 
          ControlSystem.kLBturnB,
          DriveConstants.kBackLeftChassisAngularOffset);
  private final SwerveModule m_backRight = new SwerveModule(
          ControlSystem.kRightBackDrive,
          ControlSystem.kRightBackTurn, 
          ControlSystem.kRBturnA, 
          ControlSystem.kRBturnB,
          DriveConstants.kBackRightChassisAngularOffset);

  private final ADIS16448_IMU m_imu = new ADIS16448_IMU();

  
  private final SwerveDriveKinematics m_kinematics =
        new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
   
    
        
  private final SwerveDriveOdometry m_odometry =
        new SwerveDriveOdometry(
            m_kinematics,
            new Rotation2d(m_imu.getAngle()),
            new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_backLeft.getPosition(),
              m_backRight.getPosition()
            });
 

 /** Creates a new Drive Train Subsystem. */
  public DriveTrain() {
    m_imu.reset();
  }


  @Override
  public void periodic() {
    // Update odometry?
    //System.out.printf("Odo Angle Periodic %f\n", m_imu.getAngle());
    m_odometry.update(
        Rotation2d.fromDegrees(m_imu.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
  }

  public final double getOdometryAngle()
  {
    //System.out.printf("Odo Angle Call %f\n", m_imu.getAngle());
    double iMUAngle = m_imu.getAngle();
    return iMUAngle;
  } 
  
  /**
   * Method to drive the robot using the Joystick.
   * 
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param zRot Angular rotation of the robot by twisting the Joystick.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double zRot, boolean fieldRelative) {
    double xSpeedCommanded = xSpeed;
    double ySpeedCommanded = ySpeed;
    double m_currentRotation = zRot;


    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeed;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeed;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;
    System.out.printf("Field %b, x=%f, y=%f, rot=%f\n", fieldRelative, xSpeed, ySpeed, zRot);

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_imu.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);         
  }
    
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }

  public double TurnCountFR() {
    double turningOut = m_backRight.TurnOutput();
    return turningOut;
  }

  public double DriveVelFL() {
    double driveVelFL = m_frontLeft.DriveOutput();
    return driveVelFL;
  }

  public double DriveVelFR() {
    double driveVelFR = m_frontRight.DriveOutput();
    return driveVelFR;
  }

  public double DriveVelBL() {
    double driveVelBL = m_backLeft.DriveOutput();
    return driveVelBL;
  }

  public double DriveVelBR() {
    double driveVelBR = m_backRight.DriveOutput();
    return driveVelBR;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
