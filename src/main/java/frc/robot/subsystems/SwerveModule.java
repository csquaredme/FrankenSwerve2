// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class SwerveModule extends SubsystemBase {
  /** This file is the base for the Swerve Modules driving the robot. Each swerve module will be controlled by the Drive Subsytem. */


  private final CANSparkMax m_driveMotor;
  private final MotorController m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final Encoder m_turningEncoder;

  private final SparkMaxPIDController m_drivePIDController;

  //public static final CANSparkMaxLowLevel.MotorType kBrushless;

  private final PIDController m_turningPIDController = new PIDController(1, 1, 1);

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  // Gains are for example purposes only - must be determined for your own robot!
  //private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);


   /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel CAN output for the drive motor.
   * @param turningMotorChannel CAN output for the turning motor.
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */
  public SwerveModule(
    int driveMotorChannel,
    int turningMotorChannel,
    int turningEncoderChannelA,
    int turningEncoderChannelB) {

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new WPI_VictorSPX(turningMotorChannel);

    m_driveMotor.restoreFactoryDefaults();
    
    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);
    m_drivePIDController = m_driveMotor.getPIDController();
    m_drivePIDController.setFeedbackDevice(m_driveEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_driveEncoder.setPositionConversionFactor(SwerveConstants.kDrivingEncoderPositionFactor);
    m_driveEncoder.setVelocityConversionFactor(SwerveConstants.kDrivingEncoderVelocityFactor);

    // Set the PID gains for the driving motor. 
    // May need to tune.
    m_drivePIDController.setP(SwerveConstants.driveGainP);
    m_drivePIDController.setI(SwerveConstants.driveGainI);
    m_drivePIDController.setD(SwerveConstants.driveGainD);
    m_drivePIDController.setFF(0);
    m_drivePIDController.setOutputRange(-1, 1); 


        // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    m_turningEncoder.setDistancePerPulse(2 * Math.PI / SwerveConstants.kAngleEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

  }

  
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    /**
    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();
    
    //final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    final double driveOutput  = m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);
    //final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint());


    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    */

    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    //correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
       SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
         new Rotation2d());


    var encoderRotation = new Rotation2d(m_turningEncoder.getDistance());
     // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);
    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
   // m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
   final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint()); 
   
   m_turningMotor.set(turnOutput + turnFeedforward);

    m_desiredState = desiredState;

    
  }
    
   /** Zeroes all the SwerveModule encoders. */
   public void resetEncoders() {
    m_driveEncoder.setPosition(0);
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
