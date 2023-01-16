// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.DreadbotController;
import frc.robot.util.misc.DreadbotMotor;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private DreadbotMotor frontLeftDriveMotor = new DreadbotMotor(new CANSparkMax(1, MotorType.kBrushless), "FrontLeftDrive");
  private DreadbotMotor frontLeftTurningMotor = new DreadbotMotor(new CANSparkMax(2, MotorType.kBrushless), "FrontLeftTurning");
  private CANCoder frontLeftTurningEncoder = new CANCoder(9);

  private DreadbotMotor frontRightDriveMotor = new DreadbotMotor(new CANSparkMax(7, MotorType.kBrushless), "FrontRightDrive");
  private DreadbotMotor frontRightTurningMotor = new DreadbotMotor(new CANSparkMax(8, MotorType.kBrushless), "FrontRightTurning");
  private CANCoder frontRightTurningEncoder = new CANCoder(12);

  private DreadbotMotor backLeftDriveMotor = new DreadbotMotor(new CANSparkMax(3, MotorType.kBrushless), "BackLeftDrive");
  private DreadbotMotor backLeftTurningMotor = new DreadbotMotor(new CANSparkMax(4, MotorType.kBrushless), "BackLeftTurning");
  private CANCoder backLeftTurningEncoder = new CANCoder(10);

  private DreadbotMotor backRightDriveMotor = new DreadbotMotor(new CANSparkMax(5, MotorType.kBrushless), "BackRightDrive");
  private DreadbotMotor backRightTurningMotor = new DreadbotMotor(new CANSparkMax(6, MotorType.kBrushless), "BackRightTurning");
  private CANCoder backRightTurningEncoder = new CANCoder(11);

  DreadbotController controller = new DreadbotController(0);
  
  Drivetrain drive = new Drivetrain(
    frontLeftDriveMotor,
    frontLeftTurningMotor,
    frontLeftTurningEncoder,
    frontRightDriveMotor,
    frontRightTurningMotor,
    frontRightTurningEncoder,
    backLeftDriveMotor,
    backLeftTurningMotor,
    backLeftTurningEncoder,
    backRightDriveMotor, 
    backRightTurningMotor,
    backRightTurningEncoder);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drive.drive(controller.getXAxis(), controller.getYAxis(), controller.getZAxis(), true);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private void calibrateEncoders(){
    boolean areCalibrated = false;

    while (!areCalibrated) {
      //System.out.println("Flood");
      if(Math.abs(frontLeftTurningEncoder.getAbsolutePosition() - -154.5d) <= .5){
        frontLeftTurningMotor.set(.5);
      }
      else{
        frontLeftTurningMotor.set(0);
      }

      if(Math.abs(frontRightTurningEncoder.getAbsolutePosition() - -33.0d) <= .5){
        frontRightTurningMotor.set(.1);
      }
      else{
        frontRightTurningMotor.set(0);
      }

      if(Math.abs(backLeftTurningEncoder.getAbsolutePosition() - -132.0d) <= .5){
        backLeftTurningMotor.set(.1);
      }
      else{
        backLeftTurningMotor.set(0);
      }

      if(Math.abs(backRightTurningEncoder.getAbsolutePosition() - 63.7d) <= .5){
        backRightTurningMotor.set(.1);
      }
      else{
        backRightTurningMotor.set(0);
      }

      if(
        Math.abs(frontLeftTurningEncoder.getAbsolutePosition() - -154.5d) <= .5 &&
        Math.abs(frontRightTurningEncoder.getAbsolutePosition() - -33.0d) <= .5 &&
        Math.abs(backLeftTurningEncoder.getAbsolutePosition() - -132.0d) <= .5 &&
        Math.abs(backRightTurningEncoder.getAbsolutePosition() - 63.7d) <= .5
      ){
        areCalibrated = true;
      }
    }

    frontLeftTurningEncoder.setPosition(0);
    frontRightTurningEncoder.setPosition(0);
    backLeftTurningEncoder.setPosition(0);
    backRightTurningEncoder.setPosition(0);
    System.out.println("DONE");
  }
}
