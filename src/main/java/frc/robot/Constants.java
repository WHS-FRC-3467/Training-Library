/*
 * Copyright (C) 2025 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drive.DriveConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static final boolean tuningMode = true;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static class RobotConstants {
        public static String serial;
        public static boolean isComp;
        public static boolean isAlpha;

        // TODO: Fill in with real serial number prefixes. Figure out by displaying/logging String
        // serial.
        public static final String compSerial = "0001";
        public static final String alphaSerial = "0000";

        static {
            if (Robot.isReal()) {
                // Roborio id recognition
                serial = System.getenv("serialnum");
            } else {
                serial = "3467";
            }
            RobotConstants.isComp = serial.startsWith(RobotConstants.compSerial);
            RobotConstants.isAlpha = serial.startsWith(RobotConstants.alphaSerial);
        }
    }

    public static RobotType robotType = RobotConstants.isComp ? RobotType.COMP
        : RobotConstants.isAlpha ? RobotType.ALPHA : RobotType.NONE;

    public enum RobotType {
        COMP,
        ALPHA,
        NONE
    }

    public static final class PathConstants {

        public static final double PATHGENERATION_DRIVE_TOLERANCE = Units.inchesToMeters(3.0); // 3
                                                                                               // in
                                                                                               // robot
                                                                                               // position
                                                                                               // tolerance
        public static final double PATHGENERATION_ROT_TOLERANCE_DEGREES = 5.0; // 5 degrees rotation
                                                                               // tolerance
        // Tune the maxAcceleration, maxAngularVelocityRadPerSec, and
        // maxAngularAccelerationRacPerSecSq constraints for pathfinding
        public static final PathConstraints ON_THE_FLY_PATH_CONSTRAINTS = new PathConstraints(
            DriveConstants.kSpeedAt12Volts.magnitude(),
            4.0,
            Units.degreesToRadians(540),
            Units.degreesToRadians(720));
    }
}
