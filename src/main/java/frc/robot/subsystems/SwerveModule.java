package frc.robot.subsystems;

import javax.print.CancelablePrintJob;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.misc.DreadbotMotor;

public class SwerveModule {

    private DreadbotMotor driveMotor;
    private DreadbotMotor turningMotor;
    private CANCoder turningCanCoder;

    private SparkMaxPIDController drivePIDController;
    private ProfiledPIDController turningPIDController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI / 2));

    public SwerveModule(DreadbotMotor driveMotor, DreadbotMotor turnMotor, CANCoder turningCanCoder, double canCoderOffset) {
        this.driveMotor = driveMotor;
        this.turningMotor = turnMotor;
        this.turningCanCoder = turningCanCoder;
        this.turningCanCoder.configMagnetOffset(-canCoderOffset);

        this.drivePIDController = driveMotor.getPIDController();
        driveMotor.PIDTuner();

        this.driveMotor.getEncoder().setPositionConversionFactor(SwerveConstants.WHEEL_DIAMETER * Math.PI * SwerveConstants.DRIVE_GEAR_RATIO); //convert from revolutions to meters
        this.driveMotor.getEncoder().setVelocityConversionFactor((SwerveConstants.WHEEL_DIAMETER * Math.PI * SwerveConstants.DRIVE_GEAR_RATIO) / 60);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotor.getEncoder().getVelocity(), 
            new Rotation2d(Units.degreesToRadians(turningCanCoder.getAbsolutePosition()))
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getEncoder().getPosition(), 
            new Rotation2d(Units.degreesToRadians(turningCanCoder.getAbsolutePosition()))
        );
    }

    public void resetEncoder() {
        driveMotor.resetEncoder();
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d(Units.degreesToRadians(turningCanCoder.getAbsolutePosition())));

        double turnOutput = turningPIDController.calculate(Units.degreesToRadians(turningCanCoder.getAbsolutePosition()), optimizedState.angle.getRadians());

        drivePIDController.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);
        turningMotor.setVoltage(turnOutput);
    }
}