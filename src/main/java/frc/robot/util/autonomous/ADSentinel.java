package frc.robot.util.autonomous;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Rotation;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.*;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/**
 * Implementation of AD* running locally in a background thread
 *
 * <p>I would like to apologize to anyone trying to understand this code. The implementation I
 * translated it from was much worse.
 */
public class ADSentinel implements Pathfinder {
  private static final double SMOOTHING_ANCHOR_PCT = 0.8;
  private static final double EPS = 2.5;

  private double fieldLength = 16.54;
  private double fieldWidth = 8.02;

  private double nodeSize = 0.2;

  private int nodesX = (int) Math.ceil(fieldLength / nodeSize);
  private int nodesY = (int) Math.ceil(fieldWidth / nodeSize);

  private final HashMap<GridPosition, Double> g = new HashMap<>();
  private final HashMap<GridPosition, Double> rhs = new HashMap<>();
  private final HashMap<GridPosition, Pair<Double, Double>> open = new HashMap<>();
  private final HashMap<GridPosition, Pair<Double, Double>> incons = new HashMap<>();
  private final Set<GridPosition> closed = new HashSet<>();
  private final Set<GridPosition> staticObstacles = new HashSet<>();
  private final Set<GridPosition> dynamicObstacles = new HashSet<>();
  private final Set<GridPosition> requestObstacles = new HashSet<>();

  private GridPosition requestStart;
  private Translation2d requestRealStartPos;
  private GridPosition requestGoal;
  private Translation2d requestRealGoalPos;

  private double eps;

  private final Thread planningThread;
  private boolean requestMinor = true;
  private boolean requestMajor = true;
  private boolean requestReset = true;
  private boolean newPathAvailable = false;

  private final ReadWriteLock pathLock = new ReentrantReadWriteLock();
  private final ReadWriteLock requestLock = new ReentrantReadWriteLock();

  private List<Waypoint> currentWaypoints = new ArrayList<>();
  private List<GridPosition> currentPathFull = new ArrayList<>();

  private List<Pose3d> aprilTagPoses = List.of();
  private double distanceWeight = 1.0;
  private double orientationWeight = 0.1;
  private List<Rotation2d> availableCameraOffsets = List.of(
    Rotation2d.fromRadians(Math.PI),
    Rotation2d.fromRadians(0.5 * Math.PI),
    Rotation2d.fromRadians(-0.5 * Math.PI)
  );

  /** Create a new pathfinder that runs AD* locally in a background thread */
  public ADSentinel() {
    planningThread = new Thread(this::runThread);

    requestStart = new GridPosition(0, 0);
    requestRealStartPos = Translation2d.kZero;
    requestGoal = new GridPosition(0, 0);
    requestRealGoalPos = Translation2d.kZero;

    staticObstacles.clear();
    dynamicObstacles.clear();

    File navGridFile = new File(Filesystem.getDeployDirectory(), "pathplanner/navgrid.json");
    if (navGridFile.exists()) {
      try (BufferedReader br = new BufferedReader(new FileReader(navGridFile))) {
        StringBuilder fileContentBuilder = new StringBuilder();
        String line;
        while ((line = br.readLine()) != null) {
          fileContentBuilder.append(line);
        }

        String fileContent = fileContentBuilder.toString();
        JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

        nodeSize = ((Number) json.get("nodeSizeMeters")).doubleValue();
        JSONArray grid = (JSONArray) json.get("grid");
        nodesY = grid.size();
        for (int row = 0; row < grid.size(); row++) {
          JSONArray rowArray = (JSONArray) grid.get(row);
          if (row == 0) {
            nodesX = rowArray.size();
          }
          for (int col = 0; col < rowArray.size(); col++) {
            boolean isObstacle = (boolean) rowArray.get(col);
            if (isObstacle) {
              staticObstacles.add(new GridPosition(col, row));
            }
          }
        }

        JSONObject fieldSize = (JSONObject) json.get("field_size");
        fieldLength = ((Number) fieldSize.get("x")).doubleValue();
        fieldWidth = ((Number) fieldSize.get("y")).doubleValue();
      } catch (Exception e) {
        // Do nothing, use defaults
      }
    }

    requestObstacles.clear();
    requestObstacles.addAll(staticObstacles);
    requestObstacles.addAll(dynamicObstacles);

    requestReset = true;
    requestMajor = true;
    requestMinor = true;

    newPathAvailable = false;

    planningThread.setDaemon(true);
    planningThread.setName("ADStar Planning Thread");
    planningThread.start();
  }

  /**
   * Get if a new path has been calculated since the last time a path was retrieved
   *
   * @return True if a new path is available
   */
  @Override
  public boolean isNewPathAvailable() {
    return newPathAvailable;
  }

  /**
   * Get the most recently calculated path
   *
   * @param constraints The path constraints to use when creating the path
   * @param goalEndState The goal end state to use when creating the path
   * @return The PathPlannerPath created from the points calculated by the pathfinder
   */
  @Override
  public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
    List<Waypoint> waypoints;

    pathLock.readLock().lock();
    waypoints = new ArrayList<>(currentWaypoints);
    pathLock.readLock().unlock();

    newPathAvailable = false;

    if (waypoints.size() < 2) {
      // Not enough points. Something got borked somewhere
      return null;
    }

    // Create a list to hold the zones for each waypoint (except the starting pose).
    List<PointTowardsZone> zones = new ArrayList<>();

    // Total path "length" is (number of waypoints + 1)
    double totalPathLength = waypoints.size() + 1;

    // For each waypoint (starting at index 1), calculate the best AprilTag and create a corresponding zone.
    // Don't start at index 0 because then the robot would start rotating immediately
    for (int i = 0; i < (waypoints.size() - 1); i++) {
      // Use the waypoint's anchor as the "robot" position.
      Translation2d waypointAnchor = waypoints.get(i).anchor();

      // Determine the best AprilTag for this waypoint.
      Translation2d bestTag = getBestAprilTagForPoint(waypointAnchor);
      if (bestTag == null) {
        // If no suitable tag is found, skip this waypoint.
        continue;
      }

      Rotation2d directionToBestTag = bestTag.minus(waypointAnchor).getAngle();

      // Initialize the best camera offset as null and set the smallest angle difference to the maximum value.
      Rotation2d bestOffset = null;
      double smallestAngleDifference = Double.MAX_VALUE;

      // Iterate through each available camera offset.
      for (Rotation2d offset : availableCameraOffsets) {

          // Calculate the combined rotation by adding the current offset to the last known direction toward the best tag.
          Rotation2d combinedRotation = directionToBestTag.plus(offset);

          // Compute the absolute angular difference between the combined rotation and the desired goal end rotation.
          // This difference indicates how close the camera's orientation (after applying the offset) is to the desired target orientation.
          double angleDifference = Math.abs(combinedRotation.minus(goalEndState.rotation()).getRadians());

          // If the current offset produces a smaller angular difference, update the bestOffset and smallestAngleDifference.
          if (angleDifference < smallestAngleDifference) {
              smallestAngleDifference = angleDifference;
              bestOffset = offset;
          }
      }

      // Determine the activation range for this zone.
      // Here, we take the waypoint index (which is its parameter along the path) and add fixed offsets.
      // Clamp the end value to the total path length if necessary.
      double zoneStart = i - 0.5;
      double zoneEnd = i + 0.5;
      if (zoneEnd > totalPathLength) {
        zoneEnd = totalPathLength;
      }

      // Create a new zone using the best tag's translation and rotation.
      PointTowardsZone zone = new PointTowardsZone(
          "zone_" + i,                           // name (unique for each zone)
          bestTag,              // target translation (2D)
          bestOffset,                 // rotation offset
          zoneStart,
          zoneEnd
      );

      zones.add(zone);
    }

    return new PathPlannerPath(waypoints, List.of(), zones, List.of(), List.of(), constraints, null, goalEndState, newPathAvailable);

  }

  /**
   * Sets the list of available AprilTag poses based on the provided field layout and available tag IDs.
   *
   * <p>This method loads the AprilTag field layout for the specified field, then iterates over the list
   * of available tag IDs. For each ID, it checks if a pose is defined in the layout; if so, the pose is added
   * to the list of AprilTag poses.</p>
   *
   * @param field           The field layout to use, specified by an AprilTagFields enum.
   * @param availableTagIDs A list of available AprilTag IDs for which poses are to be retrieved.
   */
  public void setAvailableTags(AprilTagFields field, List<Integer> availableTagIDs) {
    // Load the field layout based on the specified field.
    final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(field);

    // Initialize a list to store the 3D poses of the available AprilTags.
    ArrayList<Pose3d> aprilTagPoses = new ArrayList<>();

    // Iterate over each available tag ID.
    for (int availableTagID : availableTagIDs) {
        // Check if a pose exists for the current tag ID in the field layout.
        if (fieldLayout.getTagPose(availableTagID).isPresent()) {
            // If present, add the tag's Pose3d to the aprilTagPoses list.
            aprilTagPoses.add(fieldLayout.getTagPose(availableTagID).get());
        }  
    }
    
    this.aprilTagPoses = aprilTagPoses;

  }

  /**
     * Sets the list of available AprilTag poses.
     *
     * @param poses A list of Pose3d objects representing the available AprilTag poses.
     */
    public void setAvailableTags(List<Pose3d> poses) {
      this.aprilTagPoses = poses;
  }

  /**
   * Sets the available AprilTag poses using varargs.
   *
   * @param poses Varargs of Pose3d objects representing the available AprilTag poses.
   */
  public void setAvailableTags(Pose3d... poses) {
      this.aprilTagPoses = Arrays.asList(poses);
  }

  /**
   * Sets the weights for scoring when determining optimal AprilTags.
   * These weights affect how much distance and orientation contribute to the overall score.
   *
   * @param distanceWeight    The weight for the distance component.
   * @param orientationWeight The weight for the orientation component.
   */
  public void setWeights(double distanceWeight, double orientationWeight) {
      this.distanceWeight = distanceWeight;
      this.orientationWeight = orientationWeight;
  }

  /**
   * Sets the distance weight for scoring when selecting optimal tags.
   *
   * @param distanceWeight The weight to apply to the distance factor.
   */
  public void setDistanceWeight(double distanceWeight) {
      this.distanceWeight = distanceWeight;
  }

  /**
   * Sets the orientation weight for scoring when selecting optimal tags.
   *
   * @param orientationWeight The weight to apply to the orientation factor.
   */
  public void setOrientationWeight(double orientationWeight) {
      this.orientationWeight = orientationWeight;
  }

  /**
   * Sets the available camera offset rotations as a list.
   * These offsets represent different camera positions or orientations
   * that can be used to adjust the pointing direction.
   *
   * @param availableCameraOffsets A list of Rotation2d objects representing camera offsets.
   */
  public void setAvailableCameraOffsets(List<Rotation2d> availableCameraOffsets) {
      this.availableCameraOffsets = availableCameraOffsets;
  }

  /**
   * Sets the available camera offset rotations using varargs.
   * These offsets represent different camera positions or orientations
   * that can be used to adjust the pointing direction.
   *
   * @param availableCameraOffsets Varargs of Rotation2d objects representing camera offsets.
   */
  public void setAvailableCameraOffsets(Rotation2d... availableCameraOffsets) {
      this.availableCameraOffsets = Arrays.asList(availableCameraOffsets);
  }

  /**
   * Returns the best AprilTag for a given 2D position by scoring each tag based on distance and orientation.
   *
   * @param point The 2D position (from a waypoint's anchor) to compare against.
   * @return      A Translation2d of the best AprilTag or null if none is acceptable.
   */
  private Translation2d getBestAprilTagForPoint(Translation2d point) {
    Translation2d bestTagTranslation2d = null;
    double bestScore = -Double.MAX_VALUE;

    for (Pose3d tagPose3d : aprilTagPoses) {
      // Get tag's 2D translation
      Translation3d tagTranslation3d = tagPose3d.getTranslation();
      Translation2d tagTranslation2d = new Translation2d(tagTranslation3d.getX(), tagTranslation3d.getY());

      // Get tag's rotation as a Rotation2d (yaw)
      Rotation3d tagRotation3d = tagPose3d.getRotation();
      Rotation2d tagRotation2d = new Rotation2d(tagRotation3d.getMeasureZ());

      // Compute vector from tag to the given point (only XY plane)
      Translation2d diff = point.minus(tagTranslation2d);
      double distance = diff.getNorm();

      // Normalize the vector if possible
      Translation2d tagToPointDir = distance > 1e-3
          ? diff.div(distance)
          : new Translation2d(0, 0);

      // Compute the tag's forward vector from its yaw.
      double tagYaw = tagRotation2d.getRadians();
      Translation2d tagForward = new Translation2d(Math.cos(tagYaw), Math.sin(tagYaw));

      // Dot product tells us how much the tag is facing the point (1 = directly facing, -1 = away)
      double orientationScore = tagForward.getX() * tagToPointDir.getX() + tagForward.getY() * tagToPointDir.getY();

      // Skip tags that face away from the point.
      if (orientationScore < 0) {
        continue;
      }

      // Inverse distance score (closer is better)
      double distanceScore = 1.0 / (distance + 1e-3);

      // Overall score: weight the distance and orientation components.
      double overallScore = (distanceWeight * distanceScore)
                            + (orientationWeight * orientationScore);

      if (overallScore > bestScore) {
        bestScore = overallScore;
        bestTagTranslation2d = tagTranslation2d;
      }
    }

    return bestTagTranslation2d;
  }

  /**
   * Set the start position to pathfind from
   *
   * @param startPosition Start position on the field. If this is within an obstacle it will be
   *     moved to the nearest non-obstacle node.
   */
  @Override
  public void setStartPosition(Translation2d startPosition) {
    GridPosition startPos = findClosestNonObstacle(getGridPos(startPosition), requestObstacles);

    if (startPos != null && !startPos.equals(requestStart)) {
      requestLock.writeLock().lock();
      requestStart = startPos;
      requestRealStartPos = startPosition;

      requestMinor = true;
      newPathAvailable = false;
      requestLock.writeLock().unlock();
    }
  }

  /**
   * Set the goal position to pathfind to
   *
   * @param goalPosition Goal position on the field. f this is within an obstacle it will be moved
   *     to the nearest non-obstacle node.
   */
  @Override
  public void setGoalPosition(Translation2d goalPosition) {
    GridPosition gridPos = findClosestNonObstacle(getGridPos(goalPosition), requestObstacles);

    if (gridPos != null) {
      requestLock.writeLock().lock();
      requestGoal = gridPos;
      requestRealGoalPos = goalPosition;

      requestMinor = true;
      requestMajor = true;
      requestReset = true;
      newPathAvailable = false;
      requestLock.writeLock().unlock();
    }
  }

  /**
   * Set the dynamic obstacles that should be avoided while pathfinding.
   *
   * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d represents
   *     opposite corners of a bounding box.
   * @param currentRobotPos The current position of the robot. This is needed to change the start
   *     position of the path if the robot is now within an obstacle.
   */
  @Override
  public void setDynamicObstacles(
      List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
    Set<GridPosition> newObs = new HashSet<>();

    for (var obstacle : obs) {
      var gridPos1 = getGridPos(obstacle.getFirst());
      var gridPos2 = getGridPos(obstacle.getSecond());

      int minX = Math.min(gridPos1.x, gridPos2.x);
      int maxX = Math.max(gridPos1.x, gridPos2.x);

      int minY = Math.min(gridPos1.y, gridPos2.y);
      int maxY = Math.max(gridPos1.y, gridPos2.y);

      for (int x = minX; x <= maxX; x++) {
        for (int y = minY; y <= maxY; y++) {
          newObs.add(new GridPosition(x, y));
        }
      }
    }

    dynamicObstacles.clear();
    dynamicObstacles.addAll(newObs);
    requestLock.writeLock().lock();
    requestObstacles.clear();
    requestObstacles.addAll(staticObstacles);
    requestObstacles.addAll(dynamicObstacles);
    requestLock.writeLock().unlock();

    pathLock.readLock().lock();
    boolean recalculate = false;
    for (GridPosition pos : currentPathFull) {
      if (requestObstacles.contains(pos)) {
        recalculate = true;
        break;
      }
    }
    pathLock.readLock().unlock();

    if (recalculate) {
      setStartPosition(currentRobotPos);
      setGoalPosition(requestRealGoalPos);
    }
  }

  @SuppressWarnings("BusyWait")
  private void runThread() {
    while (true) {
      try {
        requestLock.readLock().lock();
        boolean reset = requestReset;
        boolean minor = requestMinor;
        boolean major = requestMajor;
        GridPosition start = requestStart;
        Translation2d realStart = requestRealStartPos;
        GridPosition goal = requestGoal;
        Translation2d realGoal = requestRealGoalPos;
        Set<GridPosition> obstacles = new HashSet<>(requestObstacles);

        // Change the request booleans based on what will be done this loop
        if (reset) {
          requestReset = false;
        }

        if (minor) {
          requestMinor = false;
        } else if (major && (eps - 0.5) <= 1.0) {
          requestMajor = false;
        }
        requestLock.readLock().unlock();

        if (reset || minor || major) {
          doWork(reset, minor, major, start, goal, realStart, realGoal, obstacles);
        } else {
          try {
            Thread.sleep(10);
          } catch (InterruptedException e) {
            throw new RuntimeException(e);
          }
        }
      } catch (Exception e) {
        // Something messed up. Reset and hope for the best
        requestLock.writeLock().lock();
        requestReset = true;
        requestLock.writeLock().unlock();
      }
    }
  }

  private void doWork(
      boolean needsReset,
      boolean doMinor,
      boolean doMajor,
      GridPosition sStart,
      GridPosition sGoal,
      Translation2d realStartPos,
      Translation2d realGoalPos,
      Set<GridPosition> obstacles) {
    if (needsReset) {
      reset(sStart, sGoal);
    }

    if (doMinor) {
      computeOrImprovePath(sStart, sGoal, obstacles);

      List<GridPosition> pathPositions = extractPath(sStart, sGoal, obstacles);
      List<Waypoint> waypoints =
          createWaypoints(pathPositions, realStartPos, realGoalPos, obstacles);

      pathLock.writeLock().lock();
      currentPathFull = pathPositions;
      currentWaypoints = waypoints;
      pathLock.writeLock().unlock();

      newPathAvailable = true;
    } else if (doMajor) {
      if (eps > 1.0) {
        eps -= 0.5;
        open.putAll(incons);

        open.replaceAll((s, v) -> key(s, sStart));
        closed.clear();
        computeOrImprovePath(sStart, sGoal, obstacles);

        List<GridPosition> pathPositions = extractPath(sStart, sGoal, obstacles);
        List<Waypoint> waypoints =
            createWaypoints(pathPositions, realStartPos, realGoalPos, obstacles);

        pathLock.writeLock().lock();
        currentPathFull = pathPositions;
        currentWaypoints = waypoints;
        pathLock.writeLock().unlock();

        newPathAvailable = true;
      }
    }
  }

  private List<GridPosition> extractPath(
      GridPosition sStart, GridPosition sGoal, Set<GridPosition> obstacles) {
    if (sGoal.equals(sStart)) {
      return new ArrayList<>();
    }

    List<GridPosition> path = new ArrayList<>();
    path.add(sStart);

    var s = sStart;

    for (int k = 0; k < 200; k++) {
      HashMap<GridPosition, Double> gList = new HashMap<>();

      for (GridPosition x : getOpenNeighbors(s, obstacles)) {
        gList.put(x, g.get(x));
      }

      Map.Entry<GridPosition, Double> min = Map.entry(sGoal, Double.POSITIVE_INFINITY);
      for (var entry : gList.entrySet()) {
        if (entry.getValue() < min.getValue()) {
          min = entry;
        }
      }
      s = min.getKey();

      path.add(s);
      if (s.equals(sGoal)) {
        break;
      }
    }

    return path;
  }

  private List<Waypoint> createWaypoints(
      List<GridPosition> path,
      Translation2d realStartPos,
      Translation2d realGoalPos,
      Set<GridPosition> obstacles) {
    if (path.isEmpty()) {
      return new ArrayList<>();
    }

    List<GridPosition> simplifiedPath = new ArrayList<>();
    simplifiedPath.add(path.get(0));
    for (int i = 1; i < path.size() - 1; i++) {
      if (!walkable(simplifiedPath.get(simplifiedPath.size() - 1), path.get(i + 1), obstacles)) {
        simplifiedPath.add(path.get(i));
      }
    }
    simplifiedPath.add(path.get(path.size() - 1));

    List<Translation2d> fieldPosPath = new ArrayList<>();
    for (GridPosition pos : simplifiedPath) {
      fieldPosPath.add(gridPosToTranslation2d(pos));
    }

    if (fieldPosPath.size() < 2) {
      return new ArrayList<>();
    }

    // Replace start and end positions with their real positions
    fieldPosPath.set(0, realStartPos);
    fieldPosPath.set(fieldPosPath.size() - 1, realGoalPos);

    List<Pose2d> pathPoses = new ArrayList<>();
    pathPoses.add(
        new Pose2d(fieldPosPath.get(0), fieldPosPath.get(1).minus(fieldPosPath.get(0)).getAngle()));
    for (int i = 1; i < fieldPosPath.size() - 1; i++) {
      Translation2d last = fieldPosPath.get(i - 1);
      Translation2d current = fieldPosPath.get(i);
      Translation2d next = fieldPosPath.get(i + 1);

      Translation2d anchor1 = current.minus(last).times(SMOOTHING_ANCHOR_PCT).plus(last);
      Rotation2d heading1 = current.minus(last).getAngle();
      Translation2d anchor2 = current.minus(next).times(SMOOTHING_ANCHOR_PCT).plus(next);
      Rotation2d heading2 = next.minus(anchor2).getAngle();

      pathPoses.add(new Pose2d(anchor1, heading1));
      pathPoses.add(new Pose2d(anchor2, heading2));
    }
    pathPoses.add(
        new Pose2d(
            fieldPosPath.get(fieldPosPath.size() - 1),
            fieldPosPath
                .get(fieldPosPath.size() - 1)
                .minus(fieldPosPath.get(fieldPosPath.size() - 2))
                .getAngle()));

    return PathPlannerPath.waypointsFromPoses(pathPoses);
  }

  private GridPosition findClosestNonObstacle(GridPosition pos, Set<GridPosition> obstacles) {
    if (!obstacles.contains(pos)) {
      return pos;
    }

    Set<GridPosition> visited = new HashSet<>();

    Queue<GridPosition> queue = new LinkedList<>(getAllNeighbors(pos));

    while (!queue.isEmpty()) {
      GridPosition check = queue.poll();
      if (!obstacles.contains(check)) {
        return check;
      }
      visited.add(check);

      for (GridPosition neighbor : getAllNeighbors(check)) {
        if (!visited.contains(neighbor) && !queue.contains(neighbor)) {
          queue.add(neighbor);
        }
      }
    }
    return null;
  }

  private boolean walkable(GridPosition s1, GridPosition s2, Set<GridPosition> obstacles) {
    int x0 = s1.x;
    int y0 = s1.y;
    int x1 = s2.x;
    int y1 = s2.y;

    int dx = Math.abs(x1 - x0);
    int dy = Math.abs(y1 - y0);
    int x = x0;
    int y = y0;
    int n = 1 + dx + dy;
    int xInc = (x1 > x0) ? 1 : -1;
    int yInc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; n--) {
      if (obstacles.contains(new GridPosition(x, y))) {
        return false;
      }

      if (error > 0) {
        x += xInc;
        error -= dy;
      } else if (error < 0) {
        y += yInc;
        error += dx;
      } else {
        x += xInc;
        y += yInc;
        error -= dy;
        error += dx;
        n--;
      }
    }

    return true;
  }

  private void reset(GridPosition sStart, GridPosition sGoal) {
    g.clear();
    rhs.clear();
    open.clear();
    incons.clear();
    closed.clear();

    for (int x = 0; x < nodesX; x++) {
      for (int y = 0; y < nodesY; y++) {
        g.put(new GridPosition(x, y), Double.POSITIVE_INFINITY);
        rhs.put(new GridPosition(x, y), Double.POSITIVE_INFINITY);
      }
    }

    rhs.put(sGoal, 0.0);

    eps = EPS;

    open.put(sGoal, key(sGoal, sStart));
  }

  private void computeOrImprovePath(
      GridPosition sStart, GridPosition sGoal, Set<GridPosition> obstacles) {
    while (true) {
      var sv = topKey();
      if (sv == null) {
        break;
      }
      var s = sv.getFirst();
      var v = sv.getSecond();

      if (comparePair(v, key(sStart, sStart)) >= 0 && rhs.get(sStart).equals(g.get(sStart))) {
        break;
      }

      open.remove(s);

      if (g.get(s) > rhs.get(s)) {
        g.put(s, rhs.get(s));
        closed.add(s);

        for (GridPosition sn : getOpenNeighbors(s, obstacles)) {
          updateState(sn, sStart, sGoal, obstacles);
        }
      } else {
        g.put(s, Double.POSITIVE_INFINITY);
        for (GridPosition sn : getOpenNeighbors(s, obstacles)) {
          updateState(sn, sStart, sGoal, obstacles);
        }
        updateState(s, sStart, sGoal, obstacles);
      }
    }
  }

  private void updateState(
      GridPosition s, GridPosition sStart, GridPosition sGoal, Set<GridPosition> obstacles) {
    if (!s.equals(sGoal)) {
      rhs.put(s, Double.POSITIVE_INFINITY);

      for (GridPosition x : getOpenNeighbors(s, obstacles)) {
        rhs.put(s, Math.min(rhs.get(s), g.get(x) + cost(s, x, obstacles)));
      }
    }

    open.remove(s);

    if (!g.get(s).equals(rhs.get(s))) {
      if (!closed.contains(s)) {
        open.put(s, key(s, sStart));
      } else {
        incons.put(s, Pair.of(0.0, 0.0));
      }
    }
  }

  private double cost(GridPosition sStart, GridPosition sGoal, Set<GridPosition> obstacles) {
    if (isCollision(sStart, sGoal, obstacles)) {
      return Double.POSITIVE_INFINITY;
    }

    return heuristic(sStart, sGoal);
  }

  private boolean isCollision(GridPosition sStart, GridPosition sEnd, Set<GridPosition> obstacles) {
    if (obstacles.contains(sStart) || obstacles.contains(sEnd)) {
      return true;
    }

    if (sStart.x != sEnd.x && sStart.y != sEnd.y) {
      GridPosition s1;
      GridPosition s2;

      if (sEnd.x - sStart.x == sStart.y - sEnd.y) {
        s1 = new GridPosition(Math.min(sStart.x, sEnd.x), Math.min(sStart.y, sEnd.y));
        s2 = new GridPosition(Math.max(sStart.x, sEnd.x), Math.max(sStart.y, sEnd.y));
      } else {
        s1 = new GridPosition(Math.min(sStart.x, sEnd.x), Math.max(sStart.y, sEnd.y));
        s2 = new GridPosition(Math.max(sStart.x, sEnd.x), Math.min(sStart.y, sEnd.y));
      }

      return obstacles.contains(s1) || obstacles.contains(s2);
    }

    return false;
  }

  private List<GridPosition> getOpenNeighbors(GridPosition s, Set<GridPosition> obstacles) {
    List<GridPosition> ret = new ArrayList<>();

    for (int xMove = -1; xMove <= 1; xMove++) {
      for (int yMove = -1; yMove <= 1; yMove++) {
        GridPosition sNext = new GridPosition(s.x + xMove, s.y + yMove);
        if (!obstacles.contains(sNext)
            && sNext.x >= 0
            && sNext.x < nodesX
            && sNext.y >= 0
            && sNext.y < nodesY) {
          ret.add(sNext);
        }
      }
    }
    return ret;
  }

  private List<GridPosition> getAllNeighbors(GridPosition s) {
    List<GridPosition> ret = new ArrayList<>();

    for (int xMove = -1; xMove <= 1; xMove++) {
      for (int yMove = -1; yMove <= 1; yMove++) {
        GridPosition sNext = new GridPosition(s.x + xMove, s.y + yMove);
        if (sNext.x >= 0 && sNext.x < nodesX && sNext.y >= 0 && sNext.y < nodesY) {
          ret.add(sNext);
        }
      }
    }
    return ret;
  }

  private Pair<Double, Double> key(GridPosition s, GridPosition sStart) {
    if (g.get(s) > rhs.get(s)) {
      return Pair.of(rhs.get(s) + eps * heuristic(sStart, s), rhs.get(s));
    } else {
      return Pair.of(g.get(s) + heuristic(sStart, s), g.get(s));
    }
  }

  private Pair<GridPosition, Pair<Double, Double>> topKey() {
    Map.Entry<GridPosition, Pair<Double, Double>> min = null;
    for (var entry : open.entrySet()) {
      if (min == null || comparePair(entry.getValue(), min.getValue()) < 0) {
        min = entry;
      }
    }

    if (min == null) {
      return null;
    }

    return Pair.of(min.getKey(), min.getValue());
  }

  private double heuristic(GridPosition sStart, GridPosition sGoal) {
    return Math.hypot(sGoal.x - sStart.x, sGoal.y - sStart.y);
  }

  private int comparePair(Pair<Double, Double> a, Pair<Double, Double> b) {
    int first = Double.compare(a.getFirst(), b.getFirst());
    if (first == 0) {
      return Double.compare(a.getSecond(), b.getSecond());
    } else {
      return first;
    }
  }

  private GridPosition getGridPos(Translation2d pos) {
    int x = (int) Math.floor(pos.getX() / nodeSize);
    int y = (int) Math.floor(pos.getY() / nodeSize);

    return new GridPosition(x, y);
  }

  private Translation2d gridPosToTranslation2d(GridPosition pos) {
    return new Translation2d(
        (pos.x * nodeSize) + (nodeSize / 2.0), (pos.y * nodeSize) + (nodeSize / 2.0));
  }

  /**
   * Represents a node in the pathfinding grid
   *
   * @param x X index in the grid
   * @param y Y index in the grid
   */
  public record GridPosition(int x, int y) implements Comparable<GridPosition> {
    @Override
    public int compareTo(GridPosition o) {
      if (x == o.x) {
        return Integer.compare(y, o.y);
      } else {
        return Integer.compare(x, o.x);
      }
    }
  }
}
