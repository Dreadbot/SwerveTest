package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.misc.DreadbotMotor;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain {
    //Double check locations
    //What is location in comparasion to front of bot 
    //Example x and y's seem to be swapped?
    private final Translation2d frontLeftLocation = new Translation2d(SwerveConstants.MODULE_OFFSET, SwerveConstants.MODULE_OFFSET);
    private final Translation2d frontRightLocation = new Translation2d(-SwerveConstants.MODULE_OFFSET, SwerveConstants.MODULE_OFFSET);
    private final Translation2d backLeftLocation = new Translation2d(SwerveConstants.MODULE_OFFSET, -SwerveConstants.MODULE_OFFSET);
    private final Translation2d backRightLocation = new Translation2d(-SwerveConstants.MODULE_OFFSET, -SwerveConstants.MODULE_OFFSET);

    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule backLeftModule;
    private SwerveModule backRightModule;

    private AHRS gyro = new AHRS(SerialPort.Port.kUSB);

    private SwerveDriveKinematics kinematics;

    private SwerveDriveOdometry odometry;

    public Drivetrain() {
        frontLeftModule = new SwerveModule(
            new DreadbotMotor(new CANSparkMax(1, MotorType.kBrushless), "Front Left Drive"),
            new DreadbotMotor(new CANSparkMax(2, MotorType.kBrushless), "Front Left Turn"),
            new CANCoder(9),
            SwerveConstants.FRONT_LEFT_ENCODER_OFFSET
        );
        // frontLeftModule.getDriveMotor().setInverted(true);
        frontRightModule = new SwerveModule(
            new DreadbotMotor(new CANSparkMax(7, MotorType.kBrushless), "Front Right Drive"),
            new DreadbotMotor(new CANSparkMax(8, MotorType.kBrushless), "Front Right Turn"),
            new CANCoder(12),
            SwerveConstants.FRONT_RIGHT_ENCODER_OFFSET
        );
        backLeftModule = new SwerveModule(
            new DreadbotMotor(new CANSparkMax(3, MotorType.kBrushless), "Back Left Drive"),
            new DreadbotMotor(new CANSparkMax(4, MotorType.kBrushless), "Back Left Turn"),
            new CANCoder(10),
            SwerveConstants.BACK_LEFT_ENCODER_OFFSET
        );
        // backLeftModule.getDriveMotor().setInverted(true);
        backRightModule = new SwerveModule(
            new DreadbotMotor(new CANSparkMax(5, MotorType.kBrushless), "Back Right Drive"),
            new DreadbotMotor(new CANSparkMax(6, MotorType.kBrushless), "Back Right Turn"),
            new CANCoder(11),
            SwerveConstants.BACK_RIGHT_ENCODER_OFFSET
        );

        gyro.reset();

        kinematics = new SwerveDriveKinematics(
            frontLeftLocation,
            backLeftLocation,
            frontRightLocation,
            backRightLocation
        );

        odometry  = new SwerveDriveOdometry(
            kinematics,
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }
        );

        frontLeftModule.putValuesToSmartDashboard("front left");
        frontRightModule.putValuesToSmartDashboard("front right");
        backLeftModule.putValuesToSmartDashboard("back left");
        backRightModule.putValuesToSmartDashboard("back right");
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative){
        SwerveModuleState[] swerveModuleStates = 
            kinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeed, ySpeed, rot)
            );
        frontLeftModule.putValuesToSmartDashboard("front left");
        frontRightModule.putValuesToSmartDashboard("front right");
        backLeftModule.putValuesToSmartDashboard("back left");
        backRightModule.putValuesToSmartDashboard("back right");

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.ATTAINABLE_MAX_SPEED);
        frontLeftModule.setDesiredState(swerveModuleStates[0]);
        frontRightModule.setDesiredState(swerveModuleStates[1]);
        backLeftModule.setDesiredState(swerveModuleStates[2]);
        backRightModule.setDesiredState(swerveModuleStates[3]);
    }

    public void updateOdometry() {
        odometry.update(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }
        );
    }
}
