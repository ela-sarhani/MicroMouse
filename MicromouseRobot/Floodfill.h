#ifndef FLOODFILL_H
#define FLOODFILL_H

#include <Arduino.h>
#include "config.h"

struct MazeCell {
  uint8_t walls;      // bitmask: WALL_NORTH | WALL_EAST | WALL_SOUTH | WALL_WEST
  uint8_t distance;    // flood-fill distance value to goal
  bool visited;        // has the robot physically visited this cell
};

// Maze representation + Floodfill pathfinding.
// Global/absolute frame: X = East axis (0..MAZE_SIZE-1), Y = North axis.
class Floodfill {
public:
  static void init(); // clears grid, sets outer border walls, seeds start cell

  // --- robot pose tracking (maintained by FSM as it moves) ---
  static void setCurrentPosition(uint8_t x, uint8_t y);
  static void setCurrentHeading(Heading h);
  static uint8_t getCurrentX();
  static uint8_t getCurrentY();
  static Heading getCurrentHeading();

  // Marks walls at the current cell based on sensor booleans relative to the
  // robot's *current heading* (front/left/right), converts to absolute frame.
  static void updateWallsFromLocalSense(bool front, bool left, bool right);

  // Runs BFS-style flood fill, recomputing maze[][].distance from the given
  // goal region outward (multi-source BFS bounded by known walls).
  static void floodFillToGoal();
  static void floodFillToStart();

  // Returns the absolute Heading the robot should move to reach the lowest
  // neighboring cell distance (classic floodfill greedy step), honoring
  // known walls. Returns HEADING_NORTH..WEST; check canMove() first.
  static Heading getNextHeading();
  static bool canMove(Heading h);

  static bool atGoal();
  static bool atStart();

  // After exploration: computes/stores the shortest known path start->goal
  // for the speed run, following distance-gradient descent.
  static void computeShortestPath();
  static uint8_t shortestPathHeadings[MAZE_SIZE * MAZE_SIZE];
  static uint16_t shortestPathLength;

#if DEBUG_PRINT_FLOODFILL
  static void printGrid();
#endif

  static MazeCell maze[MAZE_SIZE][MAZE_SIZE];

private:
  static uint8_t currentX, currentY;
  static Heading currentHeading;

  static void floodFillMultiSource(const uint8_t* goalXs, const uint8_t* goalYs, uint8_t goalCount);
  static bool wallBetween(uint8_t x, uint8_t y, Heading dir);
};

#endif // FLOODFILL_H
