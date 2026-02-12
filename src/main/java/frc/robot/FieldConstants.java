package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.robotStates;

import edu.wpi.first.math.util.Units;

public final class FieldConstants {

  // Field dimensions
  public static final double FIELD_LENGTH_Y = Units.inchesToMeters(651.22);
  public static final double FIELD_WIDTH_X = Units.inchesToMeters(317.69);

  public static final double FIELD_CENTER_X = FIELD_WIDTH_X / 2.0;
  public static final double FIELD_CENTER_Y = FIELD_LENGTH_Y / 2.0;


  // Assume none of these values are correct. Get from the field CAD model
  public static final double BLUE_MAX_Y = Units.inchesToMeters(181.56);
  public static final double BLUE_DEEP_MAX_Y = Units.inchesToMeters(90.78);
  public static final double BLUE_FRONT_MAX_Y = BLUE_MAX_Y;

  public static final double BLUE_GATE_MIN_Y = BLUE_MAX_Y;
  public static final double BLUE_GATE_MAX_Y = Units.inchesToMeters(325.61);
  
  public static final double NEUTRAL_ZONE_MIN_Y = BLUE_GATE_MAX_Y;
  public static final double NEUTRAL_ZONE_MAX_Y = Units.inchesToMeters(325.61);

  public static final double RED_GATE_MIN_Y = NEUTRAL_ZONE_MAX_Y;
  public static final double RED_GATE_MAX_Y = Units.inchesToMeters(490.39);

  public static final double RED_MIN_Y = RED_GATE_MAX_Y;
  public static final double RED_DEEP_MIN_Y = Units.inchesToMeters(490.39);
  public static final double RED_FRONT_MIN_Y = RED_MIN_Y;

  public static final double LEFT_TRENCH_MIN_X = Units.inchesToMeters(50.0);
  public static final double LEFT_TRENCH_MAX_X = Units.inchesToMeters(267.69);

  public static final double LEFT_BUMP_MIN_X = Units.inchesToMeters(50.0);
  public static final double LEFT_BUMP_MAX_X = Units.inchesToMeters(267.69);

  public static final double RIGHT_BUMP_MIN_X = Units.inchesToMeters(50.0);
  public static final double RIGHT_BUMP_MAX_X = Units.inchesToMeters(267.69);

  public static final double RIGHT_TRENCH_MIN_X = Units.inchesToMeters(50.0);
  public static final double RIGHT_TRENCH_MAX_X = Units.inchesToMeters(267.69);

  // objects in the field

  public static final double RED_OUTPOST_MIN_Y = Units.inchesToMeters(490.39);
  public static final double RED_OUTPOST_MAX_Y = Units.inchesToMeters(651.22);
  public static final double RED_OUTPOST_MIN_X = Units.inchesToMeters(50.0);
  public static final double RED_OUTPOST_MAX_X = Units.inchesToMeters(267.69);

  public static final double BLUE_OUTPOST_MIN_Y = Units.inchesToMeters(0.0);
  public static final double BLUE_OUTPOST_MAX_Y = Units.inchesToMeters(90.78);
  public static final double BLUE_OUTPOST_MIN_X = Units.inchesToMeters(50.0);
  public static final double BLUE_OUTPOST_MAX_X = Units.inchesToMeters(267.69);

  public static final double RED_TOWER_MIN_Y = Units.inchesToMeters(325.61);
  public static final double RED_TOWER_MAX_Y = Units.inchesToMeters(490.39);
  public static final double RED_TOWER_MIN_X = Units.inchesToMeters(50.0);
  public static final double RED_TOWER_MAX_X = Units.inchesToMeters(267.69);

  public static final double BLUE_TOWER_MIN_Y = Units.inchesToMeters(90.78);
  public static final double BLUE_TOWER_MAX_Y = Units.inchesToMeters(325.61);
  public static final double BLUE_TOWER_MIN_X = Units.inchesToMeters(50.0);
  public static final double BLUE_TOWER_MAX_X = Units.inchesToMeters(267.69);

  public static final double RED_HUB_MIN_Y = Units.inchesToMeters(325.61);
  public static final double RED_HUB_MAX_Y = Units.inchesToMeters(490.39);
  public static final double RED_HUB_MIN_X = Units.inchesToMeters(50.0);
  public static final double RED_HUB_MAX_X = Units.inchesToMeters(267.69);

  public static final double BLUE_HUB_MIN_Y = Units.inchesToMeters(90.78);
  public static final double BLUE_HUB_MAX_Y = Units.inchesToMeters(325.61);
  public static final double BLUE_HUB_MIN_X = Units.inchesToMeters(50.0);
  public static final double BLUE_HUB_MAX_X = Units.inchesToMeters(267.69);

  // target locations

  public static final double RED_DEPOT_CENTER_Y = Units.inchesToMeters(325.61);
  public static final double RED_DEPOT_CENTER_X = Units.inchesToMeters(158.845);

  public static final double BLUE_DEPOT_CENTER_Y = Units.inchesToMeters(325.61);
  public static final double BLUE_DEPOT_CENTER_X = Units.inchesToMeters(158.845);

  public static final double RED_HUB_TARGET_Y = Units.inchesToMeters(490.39);
  public static final double RED_HUB_TARGET_X = Units.inchesToMeters(158.845);

  public static final double BLUE_HUB_TARGET_Y = Units.inchesToMeters(90.78);
  public static final double BLUE_HUB_TARGET_X = Units.inchesToMeters(158.845);

  public static final double BLUE_LEFT_TARGET_X = Units.inchesToMeters(50.0);
  public static final double BLUE_RIGHT_TARGET_X = Units.inchesToMeters(267.69);
  public static final double BLUE_TARGET_Y = Units.inchesToMeters(90.78);

  public static final double RED_LEFT_TARGET_X = Units.inchesToMeters(50.0);
  public static final double RED_RIGHT_TARGET_X = Units.inchesToMeters(267.69);
  public static final double RED_TARGET_Y = Units.inchesToMeters(490.39);

  public enum FieldRegion {
    BLUE_DEEP_LEFT,
    BLUE_DEEP_RIGHT,
    BLUE_FRONT_LEFT,
    BLUE_FRONT_RIGHT,
    NEUTRAL_LEFT,
    NEUTRAL_RIGHT,
    RED_DEEP_LEFT,
    RED_DEEP_RIGHT,
    RED_FRONT_LEFT,
    RED_FRONT_RIGHT,
    RED_LEFT_TRENCH,
    RED_RIGHT_TRENCH,
    RED_LEFT_BUMP,
    RED_RIGHT_BUMP,
    BLUE_LEFT_TRENCH,
    BLUE_RIGHT_TRENCH,
    BLUE_RIGHT_BUMP,
    BLUE_LEFT_BUMP,
    UNKNOWN
  }

  // NEED TO CHECK IF WE'RE ON BLUE OR RED SIDE
  // public static boolean isScoringAllowed(FieldRegion region) {
  //   switch (region) {
  //     case BLUE_DEEP_LEFT:
  //     case BLUE_DEEP_RIGHT:
  //     case BLUE_FRONT_LEFT:
  //     case BLUE_FRONT_RIGHT:
  //       return true; // Scoring allowed in alliance zone
  //     default:
  //       return false; // Scoring not allowed in neutral or opponent zones
  //   }
  // }

  public static String toString(FieldRegion region) {
    switch (region) {
      case BLUE_DEEP_LEFT:
        return "Blue Deep Left";
      case BLUE_DEEP_RIGHT:
        return "Blue Deep Right";
      case BLUE_FRONT_LEFT:
        return "Blue Front Left";
      case BLUE_FRONT_RIGHT:
        return "Blue Front Right";
      case NEUTRAL_LEFT:
        return "Neutral Left";
      case NEUTRAL_RIGHT:
        return "Neutral Right";
      case BLUE_LEFT_TRENCH:
        return "Blue Left Trench";
      case BLUE_RIGHT_TRENCH:
        return "Blue Right Trench";
      case BLUE_LEFT_BUMP:
        return "Blue Left Bump";
      case BLUE_RIGHT_BUMP:
        return "Blue Right Bump";
      case RED_LEFT_BUMP:
        return "Red Left Bump";
      case RED_RIGHT_BUMP:
        return "Red Right Bump";
      case RED_LEFT_TRENCH:
        return "Red Left Trench";
      case RED_RIGHT_TRENCH:
        return "Red Right Trench";
      case RED_DEEP_LEFT:
        return "Red Deep Left";
      case RED_DEEP_RIGHT:
        return "Red Deep Right";
      case RED_FRONT_LEFT:
        return "Red Front Left";
      case RED_FRONT_RIGHT:
        return "Red Front Right";
      default:
        return "Unknown Field Region";
    }
  }

  public FieldRegion CurrentFieldState(Pose2d robotPose2D){
    double y = robotPose2D.getY();
    double x = robotPose2D.getX();

    if(y < 0 || y > FIELD_LENGTH_Y || x < 0 || x > FIELD_WIDTH_X){
      return FieldRegion.UNKNOWN; // Out of bounds
    }

    if(x < FIELD_CENTER_X){
      // Left side of the field
      if(y < BLUE_DEEP_MAX_Y){
        return FieldRegion.BLUE_DEEP_LEFT;
      } else if (y < BLUE_FRONT_MAX_Y){
        return FieldRegion.BLUE_FRONT_LEFT;
      } else if (y < BLUE_GATE_MAX_Y){
          if (x < LEFT_TRENCH_MAX_X){
            return FieldRegion.BLUE_LEFT_TRENCH;
          } else if (x < LEFT_BUMP_MAX_X){
            return FieldRegion.BLUE_LEFT_BUMP;
          }
      } else if (y < NEUTRAL_ZONE_MAX_Y){
        return FieldRegion.NEUTRAL_LEFT;
      } else if (y < RED_GATE_MAX_Y){
        if (x < LEFT_TRENCH_MAX_X){
          return FieldRegion.RED_LEFT_TRENCH;
        } else if (x < LEFT_BUMP_MAX_X){
          return FieldRegion.RED_LEFT_BUMP;
        }
      } else if (y < RED_DEEP_MIN_Y){
        return FieldRegion.RED_FRONT_LEFT;
      } else {
        return FieldRegion.RED_DEEP_LEFT;
      }
    } else {
      // Right side of the field
      if(y < BLUE_DEEP_MAX_Y){
        return FieldRegion.BLUE_DEEP_RIGHT;
      } else if (y < BLUE_FRONT_MAX_Y){
        return FieldRegion.BLUE_FRONT_RIGHT;
      } else if (y < BLUE_GATE_MAX_Y){
          if (x < RIGHT_BUMP_MAX_X){
            return FieldRegion.BLUE_RIGHT_BUMP;
          } else if (x < RIGHT_TRENCH_MAX_X){
            return FieldRegion.BLUE_RIGHT_TRENCH;
          }
      } else if (y < NEUTRAL_ZONE_MAX_Y){
        return FieldRegion.NEUTRAL_RIGHT;
      } else if (y < RED_GATE_MAX_Y){
        if (x > RIGHT_BUMP_MIN_X){
          return FieldRegion.RED_RIGHT_BUMP;
        } else if (x > RIGHT_TRENCH_MIN_X){
          return FieldRegion.RED_RIGHT_TRENCH;
        }
      } else if (y < RED_DEEP_MIN_Y){
        return FieldRegion.RED_FRONT_RIGHT;
      } else {
        return FieldRegion.RED_DEEP_RIGHT;
      }
    }
    return FieldRegion.UNKNOWN; // Default case if none of the conditions are met (this should never happen)
  }
}