package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

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
    private double kWheelRadius = .05;
    private int kEncoderResolution = 4096;

    private DreadbotMotor driveMotor;
    private DreadbotMotor turningMotor;
    private CANCoder turningEncoder;

    //Configure below variables for our bot
    private final PIDController drivePIDController = new PIDController(.0000001, 0, 0);

    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1/2, 3/2);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1/2, 0.5/2);

    //Setup trapeziod profile later on for max speeds
    private ProfiledPIDController m_turningPIDController =
         new ProfiledPIDController(
            .0000001,
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

            turningMotor.setPositionConversionFactor(2 * Math.PI/kEncoderResolution);

            //turningMotor.setPosition(0);

            //Find function to make turning PID input continuous and limited to pi to -pi
            turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
            //turningEncoder.
            //turningEncoder.setPosition(90);
    }
 
    public SwerveModulePosition getPosition() {

        return new SwerveModulePosition(
            driveMotor.getPosition(), new Rotation2d().fromDegrees(turningEncoder.getAbsolutePosition()));
    }

    int i = 0;
    public void setDesiredState(SwerveModuleState desiredState){
        if(i % 50 == 0)
            System.out.println(turningEncoder.getAbsolutePosition());
        i++;
        
        SwerveModuleState state = desiredState;
            //SwerveModuleState.optimize(desiredState, new Rotation2d().fromDegrees(turningEncoder.getAbsolutePosition()));
            
            if(i % 50 == 0)
                System.out.println("Target" + state.angle.getDegrees());

        final double driveOutput = 
            drivePIDController.calculate(driveMotor.getVelocity());

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
            m_turningPIDController.calculate(turningEncoder.getAbsolutePosition(), state.angle.getDegrees());
    
        final double turnFeedforward =
            m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    
        driveMotor.setVoltage((driveOutput + driveFeedforward) );
        turningMotor.setVoltage((turnOutput + turnFeedforward) );
    }
}