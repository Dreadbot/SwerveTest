// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.sensors.CANCoder;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.DreadbotController;
import frc.robot.util.math.DreadbotMath;
import frc.robot.util.misc.DreadbotMotor;
import frc.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Drivetrain drive = new Drivetrain();

  DreadbotController controller = new DreadbotController(0);

  HolonomicDriveController holonomicDriveController =
    new HolonomicDriveController(
      new PIDController(.5, 0, 0),
      new PIDController(.5, 0, 0),
      new ProfiledPIDController(
        1, 0, 0,
        new TrapezoidProfile.Constraints(6.28, 3.14)
      )
    );

  Trajectory trajectory;

  State goal;

  boolean runningTestAuto = true;

  private final SendableChooser<Command> sendableChooser = new SendableChooser<>();
  private final HashMap<String, Command> autonEvents = new HashMap<>();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    SmartDashboard.putNumber("Drive P", 0);
    SmartDashboard.putNumber("Drive I", 0);
    SmartDashboard.putNumber("Drive D", 0);
    SmartDashboard.putNumber("Drive FF", 0);
    SmartDashboard.putBoolean("RunTestAuton", true);
    SmartDashboard.putBoolean("RunAutonPath", false);
    initAutonChooser();
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
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = sendableChooser.getSelected();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

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
    drive.drive(
      -DreadbotMath.applyDeadbandToValue(controller.getYAxis(), 0.05) * 0.4,
      -DreadbotMath.applyDeadbandToValue(controller.getXAxis(), 0.05) * 0.4,
      -DreadbotMath.applyDeadbandToValue(controller.getZAxis(), 0.05) * 0.4,
      true
    );
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

  //Copied from WPI
  public void generateTrajectory() {
    // 2018 cross scale auto waypoints.
    var sideStart = new Pose2d( 0, 0, new Rotation2d(0));
    var crossScale = new Pose2d(0, 3,new Rotation2d(0));

    var interiorWaypoints = new ArrayList<Translation2d>();
    interiorWaypoints.add(new Translation2d(0, 1));
    interiorWaypoints.add(new Translation2d(0, 2));

    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(6), Units.feetToMeters(2));
    config.setReversed(true);

    trajectory = TrajectoryGenerator.generateTrajectory(
      sideStart,
      interiorWaypoints,
      crossScale,
      config
    );
  }

  private void initAutonChooser() {
    sendableChooser.setDefaultOption("Score, Leave, & Balance", drive.buildAuto(autonEvents, "ScoreLeaveBalance"));
    sendableChooser.addOption("Partial Link Bump", drive.buildAuto(autonEvents, "PartialLinkBump"));
    sendableChooser.addOption("Partial Link Non-Bump", drive.buildAuto(autonEvents, "PartialLinkNonBump"));
    sendableChooser.addOption("Small Straight", drive.buildAuto(autonEvents, "SmallStraight"));
    SmartDashboard.putData(sendableChooser);
  }
}
