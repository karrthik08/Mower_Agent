// Ardumower Sunray Configurable Simulation (with JSON settings)
#include "config.h"
#include "robot.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <unistd.h>
#include <fstream>
#include <ctime>
#include <cstdlib>
#include <vector>
#include <utility>
#include <algorithm>
#include <sstream>
#include <vector>

// Prevent Arduino macro conflicts before including nlohmann/json
#ifdef A0
#undef A0
#undef A1
#undef A2
#undef A3
#undef A4
#undef A5
#undef A6
#undef A7
#endif
#ifdef B0
#undef B0
#undef B1
#undef B2
#undef B3
#undef B4
#undef B5
#undef B6
#undef B7
#endif

#include <nlohmann/json.hpp>

using json = nlohmann::json;

// ---- Global simulation variables ----
double simBattery = 100.0;
double drain_rate = 0.05;
double charge_rate = 0.1;
double cutting_height = 3.0;
double grass_default_height = 5.0;
double maxX = 20.0, maxY = 20.0;
std::string scheduled_start = "00:00";
bool charging = false;
bool started = false;
const double cellResolution = 0.2; // meters per ASCII cell
const double travelSpeed = 0.25;   // meters per loop iteration
const double waypointTolerance = 0.02;
const double dockReserveWidth = 0.6;   // meters reserved for the dock area (X axis)
const double dockReserveHeight = 0.6;  // meters reserved for the dock area (Y axis)
static const double AREA_MIN_X = 0.0;
static const double AREA_MAX_X = 10.0;
static const double AREA_MIN_Y = 0.0;
static const double AREA_MAX_Y = 10.0;
static const double ROW_SPACING = 0.6;
static const double TARGET_TOL = 0.10;
double batteryLowThreshold = 10.0;
int gridWidthCells = 0;
int gridHeightCells = 0;
bool fieldInitialized = false;
int dockColsReserved = 0;
int dockRowsReserved = 0;
std::vector<std::vector<double>> grassHeight;
std::vector<std::vector<bool>> obstacleMask;
double dockPosX = 0.0;
double dockPosY = 0.0;
double posX = 0.0;
double posY = 0.0;
typedef struct {
    int currentRow;
    int movingForward;
    int finished;
    double targetX;
    double targetY;
} CoveragePatternState;
CoveragePatternState coverageState;
bool coveragePatternActive = false;
double coverageMinX = AREA_MIN_X;
double coverageMaxX = AREA_MAX_X;
double coverageMinY = AREA_MIN_Y;
double coverageMaxY = AREA_MAX_Y;

struct Obstacle {
    double x;
    double y;
};

std::vector<Obstacle> obstacles = {
    {4.0, 4.0},
    {6.0, 2.0},
    {2.0, 7.0}
};

void resetGrassHeight();
void initializeSimulationField();
void configureCoverageBounds();
void resetCoveragePattern();
void writePoseToBridge(double x, double y, double battery, const std::string& state);
bool moveTowards(double targetX, double targetY);
void trimGrassAtPosition(double x, double y);
void applyRuntimeControls();
bool readJsonFromPaths(const std::vector<std::string>& candidatePaths, json& result);
void coveragePatternInit(CoveragePatternState* s);
void coveragePatternUpdate(CoveragePatternState* s, double robotX, double robotY);
int coveragePatternIsFinished(const CoveragePatternState* s);

bool readJsonFromPaths(const std::vector<std::string>& candidatePaths, json& result) {
    for (const auto& path : candidatePaths) {
        std::ifstream file(path);
        if (!file.is_open()) continue;
        std::stringstream buffer;
        buffer << file.rdbuf();
        json parsed = json::parse(buffer.str(), nullptr, false);
        if (parsed.is_discarded()) continue;
        result = parsed;
        return true;
    }
    return false;
}

void configureCoverageBounds() {
    coverageMinX = std::clamp(AREA_MIN_X, 0.0, maxX);
    coverageMaxX = std::clamp(AREA_MAX_X, coverageMinX, maxX);
    if (coverageMaxX <= coverageMinX) {
        coverageMaxX = std::min(maxX, coverageMinX + ROW_SPACING);
    }

    double dockLimitY = dockRowsReserved * cellResolution;
    double requestedMinY = std::clamp(AREA_MIN_Y, 0.0, maxY);
    coverageMinY = std::max(dockLimitY, requestedMinY);
    coverageMaxY = std::clamp(AREA_MAX_Y, coverageMinY, maxY);
    if (coverageMaxY <= coverageMinY) {
        coverageMaxY = std::min(maxY, coverageMinY + ROW_SPACING);
    }
}

void coveragePatternInit(CoveragePatternState* s) {
    s->currentRow = 0;
    s->movingForward = 1;
    s->finished = 0;

    if (coverageMaxX <= coverageMinX || coverageMaxY <= coverageMinY) {
        s->finished = 1;
        s->targetX = coverageMinX;
        s->targetY = coverageMinY;
        return;
    }

    double startY = std::min(coverageMaxY, coverageMinY + s->currentRow * ROW_SPACING);
    s->targetX = coverageMaxX;
    s->targetY = startY;
}

int coveragePatternIsFinished(const CoveragePatternState* s) {
    return s->finished;
}

void coveragePatternUpdate(CoveragePatternState* s, double robotX, double robotY) {
    if (s->finished) {
        return;
    }

    double dx = s->targetX - robotX;
    double dy = s->targetY - robotY;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < TARGET_TOL) {
        s->currentRow++;
        double newY = coverageMinY + s->currentRow * ROW_SPACING;

        if (newY > coverageMaxY) {
            s->finished = 1;
            return;
        }

        s->movingForward = !s->movingForward;
        if (s->movingForward) {
            s->targetX = coverageMaxX;
        } else {
            s->targetX = coverageMinX;
        }
        s->targetY = newY;
    }
}

void resetCoveragePattern() {
    coveragePatternInit(&coverageState);
    coveragePatternActive = (coveragePatternIsFinished(&coverageState) == 0);
}

void trimGrassAtPosition(double x, double y) {
    if (gridWidthCells == 0 || gridHeightCells == 0) return;

    int col = std::clamp(static_cast<int>(std::floor(x / cellResolution)), 0, gridWidthCells - 1);
    int row = std::clamp(static_cast<int>(std::floor(y / cellResolution)), 0, gridHeightCells - 1);

    if (obstacleMask[row][col]) {
        return;
    }

    grassHeight[row][col] = cutting_height;
}

// ---- Load configuration from settings.json ----
void loadSettings() {
    json config;
    const std::vector<std::string> settingsPaths = {
        "settings.json",
        "sunray/settings.json",
        "../settings.json",
        "../sunray/settings.json"
    };

    if (!readJsonFromPaths(settingsPaths, config)) {
        std::cerr << "⚠️ Could not open settings.json, using defaults.\n";
        return;
    }

    if (config.contains("battery_level")) simBattery = config["battery_level"];
    if (config.contains("battery_drain_rate")) drain_rate = config["battery_drain_rate"];
    if (config.contains("charging_rate")) charge_rate = config["charging_rate"];
    if (config.contains("cutting_height_mm")) cutting_height = config["cutting_height_mm"];
    if (config.contains("grass_default_height_mm")) grass_default_height = config["grass_default_height_mm"];
    if (config.contains("max_x")) maxX = config["max_x"];
    if (config.contains("max_y")) maxY = config["max_y"];
    if (config.contains("scheduled_start")) scheduled_start = config["scheduled_start"].get<std::string>();
    if (config.contains("battery_low_threshold_percent")) {
        batteryLowThreshold = std::clamp(static_cast<double>(config["battery_low_threshold_percent"]), 0.0, 100.0);
    }

}

// ---- Check if it’s time to start mowing ----
bool isScheduledTime() {
    time_t now = time(0);
    struct tm *ltm = localtime(&now);
    char current[6];
    snprintf(current, sizeof(current), "%02d:%02d", ltm->tm_hour, ltm->tm_min);
    return (std::string(current) == scheduled_start);
}

// ---- ASCII Map Display ----
void drawAsciiMap(int mowerCellX, int mowerCellY) {
    if (gridWidthCells == 0 || gridHeightCells == 0) return;

    size_t mowableCells = 0;
    size_t cutCells = 0;
    for (int row = 0; row < gridHeightCells; row++) {
        for (int col = 0; col < gridWidthCells; col++) {
            if (obstacleMask[row][col]) continue;
            mowableCells++;
            if (grassHeight[row][col] <= cutting_height + 0.01) cutCells++;
        }
    }

    double coverage = (mowableCells == 0) ? 100.0 :
                      (100.0 * static_cast<double>(cutCells) / static_cast<double>(mowableCells));

    std::system("clear");
    std::cout << "Sunray ASCII Simulator\n";
    std::cout << "Battery: " << std::fixed << std::setprecision(1) << simBattery
              << "% | State: " << (charging ? "CHARGING" : "MOWING")
              << " | Cutting Height: " << cutting_height << "mm"
              << " | Coverage: " << std::setprecision(0) << coverage << "%\n\n";

    for (int row = gridHeightCells - 1; row >= 0; row--) {
        for (int col = 0; col < gridWidthCells; col++) {
            char symbol = '.';
            const bool isDockCell = (row < dockRowsReserved && col < dockColsReserved);
            if (isDockCell) {
                symbol = 'D';
            } else if (obstacleMask[row][col]) {
                symbol = '#';
            } else if (grassHeight[row][col] <= cutting_height + 0.01) {
                symbol = ' ';
            }

            if (col == mowerCellX && row == mowerCellY) symbol = 'M';
            std::cout << symbol;
        }
        std::cout << '\n';
    }
}

void resetGrassHeight() {
    if (gridWidthCells == 0 || gridHeightCells == 0) return;
    grassHeight.assign(gridHeightCells, std::vector<double>(gridWidthCells, grass_default_height));
}

void initializeSimulationField() {
    gridWidthCells = std::max(1, static_cast<int>(std::ceil(maxX / cellResolution)));
    gridHeightCells = std::max(1, static_cast<int>(std::ceil(maxY / cellResolution)));

    grassHeight.assign(gridHeightCells, std::vector<double>(gridWidthCells, grass_default_height));
    obstacleMask.assign(gridHeightCells, std::vector<bool>(gridWidthCells, false));

    auto clampIndex = [](int value, int maxVal) {
        value = std::max(0, value);
        return std::min(value, maxVal - 1);
    };

    for (const auto& obs : obstacles) {
        int col = clampIndex(static_cast<int>(std::round(obs.x / cellResolution)), gridWidthCells);
        int row = clampIndex(static_cast<int>(std::round(obs.y / cellResolution)), gridHeightCells);
        obstacleMask[row][col] = true;
    }

    const int dockColsTarget = std::max(1, static_cast<int>(std::ceil(dockReserveWidth / cellResolution)));
    const int dockRowsTarget = std::max(1, static_cast<int>(std::ceil(dockReserveHeight / cellResolution)));
    dockColsReserved = std::min(dockColsTarget, gridWidthCells);
    dockRowsReserved = std::min(dockRowsTarget, gridHeightCells);
    if (dockRowsReserved >= gridHeightCells && gridHeightCells > 1) {
        dockRowsReserved = gridHeightCells - 1;
    }

    for (int row = 0; row < dockRowsReserved; row++) {
        for (int col = 0; col < dockColsReserved; col++) {
            obstacleMask[row][col] = true;
        }
    }

    dockPosX = std::min(maxX, (dockColsReserved * cellResolution) / 2.0);
    dockPosY = std::min(maxY, (dockRowsReserved * cellResolution) / 2.0);
    posX = dockPosX;
    posY = dockPosY;

    configureCoverageBounds();
    resetCoveragePattern();
    fieldInitialized = true;
}

void writePoseToBridge(double x, double y, double battery, const std::string& state) {
    std::ofstream posFile("/mnt/mower_shared/vm_output/position.txt",
                          std::ios::out | std::ios::trunc);
    if (!posFile.is_open()) {
        return;
    }
    posFile << x << "," << y << "," << battery << "," << state << "\n";
}

bool moveTowards(double targetX, double targetY) {
    double dx = targetX - posX;
    double dy = targetY - posY;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance <= waypointTolerance) {
        posX = targetX;
        posY = targetY;
        return true;
    }

    if (distance < 1e-9) {
        return true;
    }

    double step = std::min(travelSpeed, distance);
    posX += (dx / distance) * step;
    posY += (dy / distance) * step;
    return (distance - step) <= waypointTolerance;
}

void applyRuntimeControls() {
    json control;
    const std::vector<std::string> controlPaths = {
        "runtime_controls.json",
        "sunray/runtime_controls.json",
        "../runtime_controls.json",
        "../sunray/runtime_controls.json"
    };

    if (!readJsonFromPaths(controlPaths, control)) return;

    if (control.contains("battery_level") && control["battery_level"].is_number()) {
        double overrideBattery = std::clamp(control["battery_level"].get<double>(), 0.0, 100.0);
        simBattery = overrideBattery;
    }

    if (control.contains("battery_low_threshold_percent") &&
        control["battery_low_threshold_percent"].is_number()) {
        batteryLowThreshold = std::clamp(
            control["battery_low_threshold_percent"].get<double>(), 0.0, 100.0);
    }

    if (control.contains("battery_drain_rate") && control["battery_drain_rate"].is_number()) {
        double overrideDrain = std::max(0.0, control["battery_drain_rate"].get<double>());
        drain_rate = overrideDrain;
    }
}

// ---- Setup ----
void setup() {
    loadSettings();
    initializeSimulationField();
    start();
    CONSOLE.println("Loaded config-based settings!");
    setOperation(OP_MOW);
}

// ---- Main Loop ----
// === Main Loop: Row-by-row mowing with obstacle avoidance ===

void loop() {
    static std::string mowerState = "MOWING";

    if (!fieldInitialized) {
        initializeSimulationField();
    }

    applyRuntimeControls();

    if (!charging) {
        simBattery = std::max(0.0, simBattery - drain_rate);
        if (simBattery <= batteryLowThreshold && mowerState == "MOWING") {
            mowerState = "RETURNING_TO_DOCK";
            coveragePatternActive = false;
        }

        if (mowerState == "MOWING") {
            if (!coveragePatternActive) {
                resetCoveragePattern();
            }

            coveragePatternUpdate(&coverageState, posX, posY);

            if (coveragePatternIsFinished(&coverageState)) {
                mowerState = "RETURNING_TO_DOCK";
                coveragePatternActive = false;
            } else {
                moveTowards(coverageState.targetX, coverageState.targetY);
                trimGrassAtPosition(posX, posY);
            }
        } else if (mowerState == "RETURNING_TO_DOCK") {
            if (moveTowards(dockPosX, dockPosY)) {
                charging = true;
                mowerState = "CHARGING";
                coveragePatternActive = false;
            }
        }
    } else {
        posX = dockPosX;
        posY = dockPosY;
        simBattery = std::min(100.0, simBattery + charge_rate);
        if (simBattery >= 100.0) {
            charging = false;
            mowerState = "MOWING";
            resetGrassHeight();
            resetCoveragePattern();
        }
    }

    int mowerCellX = 0;
    int mowerCellY = 0;
    if (gridWidthCells > 0 && gridHeightCells > 0) {
        mowerCellX = std::clamp(static_cast<int>(std::floor(posX / cellResolution)), 0, gridWidthCells - 1);
        mowerCellY = std::clamp(static_cast<int>(std::floor(posY / cellResolution)), 0, gridHeightCells - 1);
    }

    drawAsciiMap(mowerCellX, mowerCellY);
    writePoseToBridge(posX, posY, simBattery, mowerState);
    usleep(100000);
}
