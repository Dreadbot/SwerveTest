package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.misc.DreadbotMotor;

public class SwerveModule {
    //Find radius
    private double kWheelRadius = .0508; // in Meters, .0508 = 2 inches
    private int kEncoderResolution = 4096;

    private DreadbotMotor driveMotor;
    private DreadbotMotor turningMotor;

    //Configure below variables for our bot
    //private final PIDController drivePIDController = new PIDController(1, 0, 0);

    // private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    // private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    //Setup trapeziod profile later on for max speeds
    // private ProfiledPIDController m_turningPIDController =
    //      new ProfiledPIDController(
    //         1,
    //         0,
    //         0, 
    //         new TrapezoidProfile.Constraints(
    //             Math.PI, 2* Math.PI));

    public SwerveModule(DreadbotMotor driveMotor, DreadbotMotor turningMotor) {
        this.driveMotor = driveMotor;
        this.turningMotor = turningMotor;

        driveMotor.setP(.0001);
        turningMotor.setP(.0001);

        driveMotor.setIdleMode(IdleMode.kCoast);
        turningMotor.setIdleMode(IdleMode.kCoast);

        // Wheel Circumference: 2 * PI * radius (we do all our measurements in Meters)
        // Our motor encoders return position value in rotations,
        // So, Divide the circumference by the gear ratio.
        // In this case, we want to know the number of rotations of the motor to get 1 full rotation of the tires
        //      Since the gear on the motor is smaller than the gear on the wheel, we want a number less than 1
        //      so it should be something like 15/50 (50 teeth on wheel gear, 15 teeth on motor gear)
        //*** Make sure at least one of the values is a doulbe (15.0) or it will do integer math and return 0;  ***
        // double wheelCircumference = 2 * Math.PI * kWheelRadius;
        // double driveGearRatio = 14.0/50.0; // from https://www.andymark.com/products/mk4i-swerve-modules?via=Z2lkOi8vYW5keW1hcmsvV29ya2FyZWE6Ok5hdmlnYXRpb246OlNlYXJjaFJlc3VsdHMvJTdCJTIycSUyMiUzQSUyMnN3ZXJ2ZSUyMiU3RA
        // double turnGearRatio = 7.0/150.0;
        // driveMotor.setPositionConversionFactor(wheelCircumference * driveGearRatio);
        // turningMotor.setPositionConversionFactor(2 * Math.PI * turnGearRatio);

        //Find function to make turning PID input continuous and limited to pi to -pi
        // turningMotor.getPIDController().setPositionPIDWrappingMinInput(-Math.PI);
        // turningMotor.getPIDController().setPositionPIDWrappingMaxInput(Math.PI);
        // turningMotor.getPIDController().setPositionPIDWrappingEnabled(true);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getPosition(), new Rotation2d(turningMotor.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState){
        SwerveModuleState state =
            SwerveModuleState.optimize(desiredState, new Rotation2d(turningMotor.getPosition()));
            
        //final double driveOutput = 
            //driveMotor.getPIDController().setReference(state.speedMetersPerSecond, ControlType.kVelocity);

       // System.out.println("Drive output: " +driveOutput);
        //final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        // final double turnOutput =
        //     m_turningPIDController.calculate(turningMotor.getPosition(), state.angle.getRadians());
    
        // final double turnFeedforward =
        //     m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    
       // driveMotor.setVoltage(driveOutput); //+ driveFeedforward);
       driveMotor.getPIDController().setReference(state.speedMetersPerSecond * 5000, ControlType.kVelocity);
       //System.out.println(state.angle.getRadians());
       //turningMotor.getPIDController().setReference(state.angle.getDegrees(), ControlType.kPosition);
       //driveMotor.set(.5);
        //System.out.println("Velocity " + state.speedMetersPerSecond * 2);
        //turningMotor.setReference(state.angle.getRadians(), ControlType.kPosition);
        //turningMotor.setVoltage(turnOutput); //+ turnFeedforward);
    }
}