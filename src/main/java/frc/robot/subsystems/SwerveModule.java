package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.misc.DreadbotMotor;

public class SwerveModule {
    //Find radius
    private double kWheelRadius = .0508;
    private int kEncoderResolution = 4096;

    private DreadbotMotor driveMotor;
    private DreadbotMotor turningMotor;
    private CANCoder turningEncoder;
    //Configure below variables for our bot
    private final PIDController drivePIDController = new PIDController(1e-3, 0, 0);

    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.2, 0.6);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.1, 0.05);
    //Setup trapeziod profile later on for max speeds
    private ProfiledPIDController m_turningPIDController =
         new ProfiledPIDController(
            1e-2,
            0,
            0, 
            new TrapezoidProfile.Constraints(
                Math.PI, 2* Math.PI));

    public SwerveModule(
        DreadbotMotor driveMotor,
        DreadbotMotor turningMotor,
        CANCoder turningEncoder) {
            this.driveMotor = driveMotor;
            this.turningMotor = turningMotor;
            this.turningEncoder = turningEncoder;

            driveMotor.setPositionConversionFactor(2 * Math.PI * kWheelRadius / kEncoderResolution);

            turningMotor.setPositionConversionFactor(2 * Math.PI / kEncoderResolution);
            //Find function to make turning PID input continuous and limited to pi to -pi
            m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getPosition(), new Rotation2d(turningMotor.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState){
        SwerveModuleState state =
            SwerveModuleState.optimize(desiredState, new Rotation2d(turningMotor.getPosition()));
            
        final double driveOutput = 
            drivePIDController.calculate(driveMotor.getVelocity());

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
            m_turningPIDController.calculate(turningMotor.getPosition(), state.angle.getRadians());
    
        final double turnFeedforward =
            m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    
        driveMotor.setVoltage(driveOutput + driveFeedforward);
        turningMotor.setVoltage(turnOutput + turnFeedforward);
    }
}