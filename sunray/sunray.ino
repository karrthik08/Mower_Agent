// Ardumower Sunray
// Copyright...
// GPLv3...

#include "config.h"
#include "robot.h"

// Access real battery object from battery.cpp
extern Battery battery;

#include <iostream>
#include <iomanip>
#include <cmath>
#include <unistd.h>
#include <fstream>

// Draw ASCII map
void drawAsciiMap(double x, double y) {
    const int size = 20;
    const int scale = 1;
    const char mower = 'M';

    std::system("clear");
    std::cout << "Sunray ASCII Simulator\n";
    std::cout << "Use keys: o=obstacle, r=rain, l=lowbat, Ctrl+C=exit\n\n";

    for (int j = size - 1; j >= 0; j--) {
        for (int i = 0; i < size; i++) {
            int px = static_cast<int>(std::round(x * scale));
            int py = static_cast<int>(std::round(y * scale));

            if (i == px && j == py)
                std::cout << mower;
            else
                std::cout << '.';
        }
        std::cout << '\n';
    }

    // Print position + battery %
    std::cout << "\nRobot position (x,y): "
              << std::fixed << std::setprecision(2)
              << x << ", " << y << std::endl;

    std::cout << "Battery: "
              << std::fixed << std::setprecision(0)
              << battery.distanceSOC << "%\n";
}


// Setup
void setup() {
    start(); // initialize real robot core (stateEstimator, battery, ops, etc.)

    battery.distanceSOC = 100;  // start full battery

    CONSOLE.println("Forcing mower to start in MowOp mode...");
    setOperation(OP_MOW);
}


// Main simulation loop
void loop() {
    static double x = 0.0, y = 0.0;
    static double vx = 0.15, vy = 0.10;
    static double maxX = 10.0, maxY = 10.0;

    // 1. Move robot
    x += vx;
    y += vy;

    // 2. Battery drain based on movement
    double distance = std::sqrt(vx*vx + vy*vy);
    battery.updateByDistance(distance);

    // 3. Bounce boundaries
    if (x > maxX || x < 0) vx = -vx;
    if (y > maxY || y < 0) vy = -vy;

    // 4. Auto-dock when SOC <= 30%
    if (battery.distanceSOC <= 30.0) {
        std::cout << "\nLOW BATTERY → Returning to HOME (0,0)...\n";

        // Stop normal wandering
        vx = 0;
        vy = 0;

        // Move robot toward home
        x -= (x * 0.1);
        y -= (y * 0.1);

        // Check if close to home
        if (x < 0.5 && y < 0.5) {
            x = 0; 
            y = 0;
            std::cout << "Docked at HOME.\n";
            drawAsciiMap(x, y);
            usleep(500000);
            return;
        }
    }

    // 5. Draw ASCII grid
    drawAsciiMap(x, y);

    // 6. Output position for external visualizers
    std::ofstream posFile("position.txt");
    if (posFile.is_open()) {
        posFile << x << "," << y;
        posFile.close();
    }

    // Slow down simulation
    usleep(100000);
}