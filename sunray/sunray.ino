// Ardumower Sunray
// (header comments omitted for brevity)

#include "config.h"
#include "robot.h"

// Access real battery object from battery.cpp
extern Battery battery;

#include <iostream>
#include <iomanip>
#include <cmath>
#include <unistd.h>
#include <fstream>

// ----------------------------------------------------------
// Grid settings
// ----------------------------------------------------------
const int    GRID_SIZE   = 20;   // 20 x 20 cells
const double GRID_SCALE  = 1.0;  // 1 meter per cell
bool visited[GRID_SIZE][GRID_SIZE] = {false};

// ----------------------------------------------------------
// Compute coverage (%) from visited[][]
// ----------------------------------------------------------
double computeCoverage() {
  int coveredCount = 0;
  for (int r = 0; r < GRID_SIZE; r++) {
    for (int c = 0; c < GRID_SIZE; c++) {
      if (visited[r][c]) coveredCount++;
    }
  }
  return (double)coveredCount / (GRID_SIZE * GRID_SIZE) * 100.0;
}

// ----------------------------------------------------------
// Draw ASCII map + coverage + battery
// ----------------------------------------------------------
void drawAsciiMap(double x, double y) {
  const char mower = 'M';

  // compute grid cell for robot
  int px = static_cast<int>(std::round(x * GRID_SCALE));
  int py = static_cast<int>(std::round(y * GRID_SCALE));

  // clamp to grid and mark visited
  if (px >= 0 && px < GRID_SIZE && py >= 0 && py < GRID_SIZE) {
    visited[py][px] = true;
  }

  std::system("clear");
  std::cout << "Sunray ASCII Simulator\n";
  std::cout << "Use keys: o=obstacle, r=rain, l=lowbat, Ctrl+C=exit\n\n";

  for (int j = GRID_SIZE - 1; j >= 0; j--) {
    for (int i = 0; i < GRID_SIZE; i++) {
      if (i == px && j == py) {
        std::cout << mower;          // robot here
      } else if (visited[j][i]) {
        std::cout << '*';            // already covered
      } else {
        std::cout << '.';            // not visited
      }
    }
    std::cout << '\n';
  }

  double percent = computeCoverage();

  std::cout << "\nCoverage: " << std::fixed << std::setprecision(1)
            << percent << "%\n";

  std::cout << "Robot position (x,y): " << std::fixed << std::setprecision(2)
            << x << ", " << y << std::endl;

  std::cout << "Battery: " << std::fixed << std::setprecision(0)
            << battery.distanceSOC << "%\n";
}

// ----------------------------------------------------------
// Setup
// ----------------------------------------------------------
void setup() {
  start(); // initialize real robot core (stateEstimator, battery, ops, etc.)
  battery.distanceSOC = 100;  // start full battery

  CONSOLE.println("Forcing mower to start in MowOp mode...");
  setOperation(OP_MOW);
}

// ----------------------------------------------------------
// Main simulation loop
// ----------------------------------------------------------
void loop() {
  static double x = 0.0, y = 0.0;

  // FIELD SIZE: sweep full 20x20 grid (0..19)
  static const double maxX = GRID_SIZE - 1; // 19
  static const double maxY = GRID_SIZE - 1; // 19

  // serpentine sweep state
  static bool   goingRight = true;  // true = move +x, false = move -x
  static double rowStep    = 0.5;   // 1 cell between rows
  static double speed      = 0.15;   // 1 cell per step

  // remember old position for distance calc
  double prevX = x;
  double prevY = y;

  // ------------------------------------------------------
  // 1. Decide: still mowing or done (100% coverage)?
  // ------------------------------------------------------
  double coverage = computeCoverage();
  bool coverageDone = (coverage >= 99.9);  // treat as 100%

  if (coverageDone) {
    // -------------------------
    // Go home after full coverage
    // -------------------------
    std::cout << "\nCOVERAGE 100% → Returning to HOME (0,0)...\n";

    // simple proportional move toward home
    x -= x * 0.2;
    y -= y * 0.2;

    // Check if close to home
    if (std::fabs(x) < 0.1 && std::fabs(y) < 0.1) {
      x = 0.0;
      y = 0.0;
      std::cout << "Docked at HOME after full coverage.\n";
      drawAsciiMap(x, y);
      usleep(500000);
      return;
    }

  } else {
    // -------------------------
    // Normal serpentine coverage
    // -------------------------
    if (goingRight) {
      x += speed;
      if (x >= maxX) {
        x = maxX;
        y += rowStep;
        goingRight = false;
      }
    } else {
      x -= speed;
      if (x <= 0.0) {
        x = 0.0;
        y += rowStep;
        goingRight = true;
      }
    }

    // clamp y so we don't walk off the grid
    if (y > maxY) {
      y = maxY;
    }
  }

  // ------------------------------------------------------
  // 2. Battery drain based on movement  (info only now)
  // ------------------------------------------------------
  double dx = x - prevX;
  double dy = y - prevY;
  double distance = std::sqrt(dx * dx + dy * dy);
  battery.updateByDistance(distance);

  // ------------------------------------------------------
  // 3. Draw ASCII grid + coverage + battery
  // ------------------------------------------------------
  drawAsciiMap(x, y);

  // ------------------------------------------------------
  // 4. Output position for external visualizers (optional)
  // ------------------------------------------------------
  std::ofstream posFile("position.txt");
  if (posFile.is_open()) {
    posFile << x << "," << y;
    posFile.close();
  }

  // ------------------------------------------------------
  // 5. Slow down simulation (0.25s per step)
  // ------------------------------------------------------
  usleep(100000);
}