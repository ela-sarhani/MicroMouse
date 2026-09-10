#include "Floodfill.h"

MazeCell Floodfill::maze[MAZE_SIZE][MAZE_SIZE];
uint8_t Floodfill::currentX = MAZE_START_X;
uint8_t Floodfill::currentY = MAZE_START_Y;
Heading Floodfill::currentHeading = HEADING_NORTH;

uint8_t Floodfill::shortestPathHeadings[MAZE_SIZE * MAZE_SIZE];
uint16_t Floodfill::shortestPathLength = 0;

// Direction deltas indexed by Heading (N,E,S,W)
static const int8_t DX[4] = { 0,  1,  0, -1 };
static const int8_t DY[4] = { 1,  0, -1,  0 };
static const uint8_t WALL_BIT[4] = { WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST };

// -----------------------------------------------------------------------------

void Floodfill::init() {
  for (uint8_t x = 0; x < MAZE_SIZE; x++) {
    for (uint8_t y = 0; y < MAZE_SIZE; y++) {
      maze[x][y].walls = 0;
      maze[x][y].distance = 255;
      maze[x][y].visited = false;
    }
  }

  // Outer border walls.
  for (uint8_t x = 0; x < MAZE_SIZE; x++) {
    maze[x][0].walls              |= WALL_SOUTH;
    maze[x][MAZE_SIZE - 1].walls  |= WALL_NORTH;
  }
  for (uint8_t y = 0; y < MAZE_SIZE; y++) {
    maze[0][y].walls              |= WALL_WEST;
    maze[MAZE_SIZE - 1][y].walls  |= WALL_EAST;
  }

  currentX = MAZE_START_X;
  currentY = MAZE_START_Y;
  currentHeading = HEADING_NORTH;
  maze[currentX][currentY].visited = true;

  floodFillToGoal();
}

void Floodfill::setCurrentPosition(uint8_t x, uint8_t y) { currentX = x; currentY = y; maze[x][y].visited = true; }
void Floodfill::setCurrentHeading(Heading h) { currentHeading = h; }
uint8_t Floodfill::getCurrentX() { return currentX; }
uint8_t Floodfill::getCurrentY() { return currentY; }
Heading Floodfill::getCurrentHeading() { return currentHeading; }

bool Floodfill::wallBetween(uint8_t x, uint8_t y, Heading dir) {
  return (maze[x][y].walls & WALL_BIT[dir]) != 0;
}

void Floodfill::updateWallsFromLocalSense(bool front, bool left, bool right) {
  // Map robot-relative (front/left/right) to absolute headings based on
  // the current heading.
  Heading absFront = currentHeading;
  Heading absLeft   = (Heading)((currentHeading + 3) % 4); // -90 deg
  Heading absRight   = (Heading)((currentHeading + 1) % 4); // +90 deg

  auto setWall = [&](Heading dir, bool present) {
    if (!present) return; // only ever *add* known walls; open sides stay "unknown/open"
    maze[currentX][currentY].walls |= WALL_BIT[dir];
    int nx = currentX + DX[dir];
    int ny = currentY + DY[dir];
    if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
      maze[nx][ny].walls |= WALL_BIT[(dir + 2) % 4]; // mutual wall on neighbor's opposite side
    }
  };

  setWall(absFront, front);
  setWall(absLeft, left);
  setWall(absRight, right);
}

// Multi-source BFS flood fill (classic micromouse algorithm) using a simple
// array-based FIFO queue (avoids STL for predictable RAM usage on ESP32).
void Floodfill::floodFillMultiSource(const uint8_t* goalXs, const uint8_t* goalYs, uint8_t goalCount) {
  static uint8_t queueX[MAZE_SIZE * MAZE_SIZE];
  static uint8_t queueY[MAZE_SIZE * MAZE_SIZE];
  uint16_t head = 0, tail = 0;

  for (uint8_t x = 0; x < MAZE_SIZE; x++)
    for (uint8_t y = 0; y < MAZE_SIZE; y++)
      maze[x][y].distance = 255;

  for (uint8_t i = 0; i < goalCount; i++) {
    maze[goalXs[i]][goalYs[i]].distance = 0;
    queueX[tail] = goalXs[i];
    queueY[tail] = goalYs[i];
    tail++;
  }

  while (head < tail) {
    uint8_t x = queueX[head];
    uint8_t y = queueY[head];
    head++;
    uint8_t curDist = maze[x][y].distance;

    for (uint8_t d = 0; d < 4; d++) {
      if (wallBetween(x, y, (Heading)d)) continue; // wall blocks this neighbor
      int nx = x + DX[d];
      int ny = y + DY[d];
      if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;
      if (maze[nx][ny].distance > curDist + 1) {
        maze[nx][ny].distance = curDist + 1;
        queueX[tail] = (uint8_t)nx;
        queueY[tail] = (uint8_t)ny;
        tail++;
      }
    }
  }

#if DEBUG_PRINT_FLOODFILL
  printGrid();
#endif
}

void Floodfill::floodFillToGoal() {
  uint8_t gx[4], gy[4];
  uint8_t n = 0;
  for (uint8_t x = MAZE_GOAL_X_MIN; x <= MAZE_GOAL_X_MAX; x++)
    for (uint8_t y = MAZE_GOAL_Y_MIN; y <= MAZE_GOAL_Y_MAX; y++) {
      gx[n] = x; gy[n] = y; n++;
    }
  floodFillMultiSource(gx, gy, n);
}

void Floodfill::floodFillToStart() {
  uint8_t gx[1] = { MAZE_START_X };
  uint8_t gy[1] = { MAZE_START_Y };
  floodFillMultiSource(gx, gy, 1);
}

bool Floodfill::canMove(Heading h) {
  if (wallBetween(currentX, currentY, h)) return false;
  int nx = currentX + DX[h];
  int ny = currentY + DY[h];
  return (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE);
}

Heading Floodfill::getNextHeading() {
  uint8_t bestDist = 255;
  Heading bestHeading = currentHeading;
  bool found = false;

  for (uint8_t d = 0; d < 4; d++) {
    if (!canMove((Heading)d)) continue;
    int nx = currentX + DX[d];
    int ny = currentY + DY[d];
    uint8_t dist = maze[nx][ny].distance;
    if (!found || dist < bestDist) {
      bestDist = dist;
      bestHeading = (Heading)d;
      found = true;
    }
  }
  return bestHeading; // caller should verify with canMove() if paranoid
}

bool Floodfill::atGoal() {
  return currentX >= MAZE_GOAL_X_MIN && currentX <= MAZE_GOAL_X_MAX &&
         currentY >= MAZE_GOAL_Y_MIN && currentY <= MAZE_GOAL_Y_MAX;
}

bool Floodfill::atStart() {
  return currentX == MAZE_START_X && currentY == MAZE_START_Y;
}

void Floodfill::computeShortestPath() {
  floodFillToGoal();

  shortestPathLength = 0;
  uint8_t x = MAZE_START_X, y = MAZE_START_Y;
  uint16_t guard = 0;

  while (!(x >= MAZE_GOAL_X_MIN && x <= MAZE_GOAL_X_MAX && y >= MAZE_GOAL_Y_MIN && y <= MAZE_GOAL_Y_MAX)) {
    uint8_t bestDist = 255;
    Heading bestDir = HEADING_NORTH;
    bool found = false;

    for (uint8_t d = 0; d < 4; d++) {
      if (maze[x][y].walls & WALL_BIT[d]) continue;
      int nx = x + DX[d];
      int ny = y + DY[d];
      if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;
      if (maze[nx][ny].distance < bestDist) {
        bestDist = maze[nx][ny].distance;
        bestDir = (Heading)d;
        found = true;
      }
    }

    if (!found) break; // dead end / disconnected - shouldn't happen post-exploration
    shortestPathHeadings[shortestPathLength++] = bestDir;
    x += DX[bestDir];
    y += DY[bestDir];

    guard++;
    if (guard > MAZE_SIZE * MAZE_SIZE) break; // safety against infinite loop
  }
}

#if DEBUG_PRINT_FLOODFILL
void Floodfill::printGrid() {
  Serial.println(F("--- Floodfill distances (Y descending) ---"));
  for (int y = MAZE_SIZE - 1; y >= 0; y--) {
    for (uint8_t x = 0; x < MAZE_SIZE; x++) {
      if (maze[x][y].distance == 255) Serial.print(F(" .. "));
      else {
        if (maze[x][y].distance < 10) Serial.print(' ');
        Serial.print(maze[x][y].distance);
        Serial.print(F("  "));
      }
    }
    Serial.println();
  }
}
#endif
