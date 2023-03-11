// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveConstants {
    public static final double ATTAINABLE_MAX_SPEED = 0.5;
    public static final double MODULE_OFFSET = Units.inchesToMeters(13.0) / 2; // 13 inches between wheels, half to meet at center
    // Encoder offsets are in degrees, not radians
    public static final double FRONT_LEFT_ENCODER_OFFSET = -46.0;
    public static final double FRONT_RIGHT_ENCODER_OFFSET = 128;
    public static final double BACK_LEFT_ENCODER_OFFSET = -66;
    public static final double BACK_RIGHT_ENCODER_OFFSET = 39.0;
    public static final double DRIVE_GEAR_RATIO = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double TURN_GEAR_RATIO = 150 / 7;
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
  }
}
