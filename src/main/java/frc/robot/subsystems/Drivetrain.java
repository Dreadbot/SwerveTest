package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.util.misc.DreadbotMotor;
import edu.wpi.first.wpilibj.SerialPort;

public class Drivetrain {
    //Double check locations
    //What is location in comparasion to front of bot 
    //Example x and y's seem to be swapped?
    private final Translation2d frontLeftLocation = new Translation2d(0.15875, 0.15875);
    private final Translation2d frontrightLocation = new Translation2d(-0.15875, 0.15875);
    private final Translation2d backLeftLocation = new Translation2d(0.15875, -0.15875);
    private final Translation2d backRightLocation = new Translation2d(-0.15875, -0.15875);

    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule backLeftModule;
    private SwerveModule backRightModule;

    private AHRS gyro = new AHRS(SerialPort.Port.kUSB);

    private SwerveDriveKinematics kinematics;

    private SwerveDriveOdometry odometry;

    public Drivetrain(
        DreadbotMotor frontLeftDriveMotor,
        DreadbotMotor frontLeftTurningMotor,
        CANCoder frontLeftTurningEncoder,
        DreadbotMotor frontRightDriveMotor,
        DreadbotMotor frontRightTurningMotor,
        CANCoder frontRightTurningEncoder,
        DreadbotMotor backLeftDriveMotor,
        DreadbotMotor backLeftTurningMotor,
        CANCoder backLeftTurningEncoder,
        DreadbotMotor backRightDriveMotor,
        DreadbotMotor backRightTurningMotor,
        CANCoder backRightTurningEncoder
        ){

        frontLeftModule = new SwerveModule(frontLeftDriveMotor, frontLeftTurningMotor, frontLeftTurningEncoder);
        frontRightModule = new SwerveModule(frontRightDriveMotor, frontRightTurningMotor, frontRightTurningEncoder);
        backLeftModule = new SwerveModule(backLeftDriveMotor, backLeftTurningMotor, backLeftTurningEncoder);
        backRightModule = new SwerveModule(backRightDriveMotor, backRightTurningMotor, backRightTurningEncoder);
        gyro.reset();

        odometry  = 
            new SwerveDriveOdometry(
                kinematics,
                gyro.getRotation2d(),
                new SwerveModulePosition[] {
                    frontLeftModule.getPosition(),
                    frontRightModule.getPosition(),
                    backLeftModule.getPosition(),
                    backRightModule.getPosition()
                });

        kinematics = 
        new SwerveDriveKinematics(
            frontLeftLocation,
            frontrightLocation,
            backLeftLocation,
            backRightLocation
        );
        
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative){
        var swerveModuleStates = 
            kinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeed, ySpeed, rot)
            );

        System.out.println(swerveModuleStates[0].speedMetersPerSecond);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 3.0);
        frontLeftModule.setDesiredState(swerveModuleStates[0]);
        frontRightModule.setDesiredState(swerveModuleStates[1]);
        backLeftModule.setDesiredState(swerveModuleStates[2]);
        backRightModule.setDesiredState(swerveModuleStates[3]);

        System.out.println(swerveModuleStates[0].speedMetersPerSecond);
    }
}
