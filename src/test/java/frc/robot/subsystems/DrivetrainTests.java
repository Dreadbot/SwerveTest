// package frc.robot.subsystems;

// import com.kauailabs.navx.frc.AHRS;

// import com.revrobotics.SparkMaxPIDController;
// import frc.robot.util.misc.DreadbotMotor;
// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.Test;
// import org.mockito.Mockito;

// import static org.junit.jupiter.api.Assertions.assertEquals;
// import static org.mockito.Mockito.mock;

// public class DrivetrainTests {

//     private DreadbotMotor frontLeftDriveMotor;
//     private DreadbotMotor frontLeftTurningMotor;
//     private DreadbotMotor frontRightDriveMotor;
//     private DreadbotMotor frontRightTurningMotor;
//     private DreadbotMotor backLeftDriveMotor;
//     private DreadbotMotor backLeftTurningMotor;
//     private DreadbotMotor backRightDriveMotor;
//     private DreadbotMotor backRightTurningMotor;
//     private Drivetrain drivetrain;

//     @BeforeEach
//     public void setup() {
//         frontLeftDriveMotor = Mockito.mock(DreadbotMotor.class);
//         frontLeftTurningMotor = Mockito.mock(DreadbotMotor.class);
//         frontRightDriveMotor = Mockito.mock(DreadbotMotor.class);
//         frontRightTurningMotor = Mockito.mock(DreadbotMotor.class);
//         backLeftDriveMotor = Mockito.mock(DreadbotMotor.class);
//         backLeftTurningMotor = Mockito.mock(DreadbotMotor.class);
//         backRightDriveMotor = Mockito.mock(DreadbotMotor.class);
//         backRightTurningMotor = Mockito.mock(DreadbotMotor.class);

//         SparkMaxPIDController mockPIDController = mock(SparkMaxPIDController.class);
//         Mockito.when(frontLeftDriveMotor.getPIDController()).thenReturn(mockPIDController);
//         Mockito.when(frontLeftTurningMotor.getPIDController()).thenReturn(mockPIDController);
//         Mockito.when(frontRightDriveMotor.getPIDController()).thenReturn(mockPIDController);
//         Mockito.when(frontRightTurningMotor.getPIDController()).thenReturn(mockPIDController);
//         Mockito.when(backLeftDriveMotor.getPIDController()).thenReturn(mockPIDController);
//         Mockito.when(backLeftTurningMotor.getPIDController()).thenReturn(mockPIDController);
//         Mockito.when(backRightDriveMotor.getPIDController()).thenReturn(mockPIDController);
//         Mockito.when(backRightTurningMotor.getPIDController()).thenReturn(mockPIDController);

//         drivetrain = new Drivetrain(
//                 frontLeftDriveMotor, frontLeftTurningMotor,
//                 frontRightDriveMotor, frontRightTurningMotor,
//                 backLeftDriveMotor, backLeftTurningMotor,
//                 backRightDriveMotor, backRightTurningMotor
//         );
//     }

//     @Test
//     public void testDriveStopped() {
//         drivetrain.drive(0.0, 0.0, 0.0, false);
//         validateModuleStates(0.0, 0);
//     }

//     @Test
//     public void testDriveForward() {
//         drivetrain.drive(0.0, 1.0, 0.0, false);
//         validateModuleStates(1.0, 90);
//     }

//     @Test
//     public void testDriveForwardRight() {
//         drivetrain.drive(1.0, 1.0, 0.0, false);
//         validateModuleStates(1.4142135623730951, 45.0);
//     }

// //    @Test
// //    public void testDriveWithRotate() {
// //        drivetrain.drive(0.0, 1.0, 1.0, false);
//         // The values are different for each module
//         // so can't use validateModuleStates method, but I don't feel like typing all the checks in right now...
// //        validateModuleStates(1.1695739074551894, 97.80101896843503);
// //    }

//     private void validateModuleStates(double speed, double angle){
//         assertEquals(speed, drivetrain.swerveModuleStates[0].speedMetersPerSecond);
//         assertEquals(angle, drivetrain.swerveModuleStates[0].angle.getDegrees());
//         assertEquals(speed, drivetrain.swerveModuleStates[1].speedMetersPerSecond);
//         assertEquals(angle, drivetrain.swerveModuleStates[1].angle.getDegrees());
//         assertEquals(speed, drivetrain.swerveModuleStates[2].speedMetersPerSecond);
//         assertEquals(angle, drivetrain.swerveModuleStates[2].angle.getDegrees());
//         assertEquals(speed, drivetrain.swerveModuleStates[3].speedMetersPerSecond);
//         assertEquals(angle, drivetrain.swerveModuleStates[3].angle.getDegrees());
//     }
// }