#pragma once
#include <array>

#include <util.hpp>

static const float defaultMoverWeight_kg = 2.0; // 2.0 in auto-calibration-simulation-for-hangprinter

class Flex {
private:
  size_t numAnchors = 4;
  // lastOnTop, four below
  std::array<Point, 5> fiveAnchors = {
      {{16.4, -1610.98, -131.53}, {1314.22, 128.14, -121.28}, {-15.73, 1415.61, -121.82}, {-1211.62, 18.14, -111.18}, {10.0, -10.0, 2299.83}}};
      // Version that fills more of the plotted rectangle
      //{{-1610.1, -1610.98, -131.53}, {1610.22, -1610.3, -121.28}, {1610.1, 1610.61, -121.82}, {-1610.62, 1610.14, -111.18}, {10.0, -10.0, 2299.83}}};
  // lastOnTop, three below
  std::array<Point, 4> fourAnchors = {
      {{16.4, -1610.98, -131.53}, {1314.22, 1268.14, -121.28}, {-1415.73, 707.61, -121.82}, {0.0, 0.0, 2299.83}}};
  std::array<Point, 3> threeAnchors = {
      {{0.0f, -1900.0f, 0.0f}, {1645.448f, 949.99999f, 0.0f}, {-1645.448f, 949.99999f, 0.0f}}};
  // Most defaults are set in the constructor
  std::array<float, HANGPRINTER_MAX_ANCHORS> spoolRadii = { 0.0F };
  std::array<float, HANGPRINTER_MAX_ANCHORS> spoolRadiiSq = { 0.0F };
  std::array<size_t, HANGPRINTER_MAX_ANCHORS> mechanicalAdvantage = { 0 };
  std::array<size_t, HANGPRINTER_MAX_ANCHORS> fullStepsPerMotorRev = { 0 };
  std::array<size_t, HANGPRINTER_MAX_ANCHORS> spoolGearTeeth = { 0 };
  std::array<size_t, HANGPRINTER_MAX_ANCHORS> motorGearTeeth = { 0 };
  std::array<size_t, HANGPRINTER_MAX_ANCHORS> linesPerSpool = { 0 };
  //float spoolBuildupFactor = 0.128181;
  float spoolBuildupFactor = 0.043003; // 0.043003 in auto-calibration-simulation-for-hangprinter
  float microsteps = 1.0;
  std::array<Point, HANGPRINTER_MAX_ANCHORS> anchors = {{ 0.0F, 0.0F, 0.0F }};
  float moverWeight_kg{defaultMoverWeight_kg};
  float springKPerUnitLength = 20000; // Garda 1.1 is somewhere in the range [20000, 100000] N/m.
  std::array<float, HANGPRINTER_MAX_ANCHORS> minPlannedForce = { 0.0F };
  std::array<float, HANGPRINTER_MAX_ANCHORS> maxPlannedForce = { 0.0F };
  std::array<float, HANGPRINTER_MAX_ANCHORS> distancesOrigin = { 0.0F };
  std::array<float, HANGPRINTER_MAX_ANCHORS> guyWireLengths = { 0.0F };
  std::array<float, HANGPRINTER_MAX_ANCHORS> springKsOrigin = { 0.0F };
  std::array<float, HANGPRINTER_MAX_ANCHORS> relaxedSpringLengthsOrigin = { 0.0F };
  std::array<float, HANGPRINTER_MAX_ANCHORS> relaxedSpringLengthsOriginMatrix = { 0.0F };
  std::array<float, HANGPRINTER_MAX_ANCHORS> k0 = { 0.0F };
  std::array<float, HANGPRINTER_MAX_ANCHORS> k2 = { 0.0F };
  float fOrigin[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
  float fOriginMatrix[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
  float targetForce_Newton = 0.0F;

  void Initk0k2();
  void InitOrigin();
  float SpringK(float const springLength) const noexcept;
  void StaticForces(Point const &machinePos, float F[HANGPRINTER_MAX_ANCHORS], bool print = false) const noexcept;
  void StaticForcesTetrahedron(Point const &machinePos, float F[HANGPRINTER_MAX_ANCHORS], bool print = false) const noexcept;
  void StaticForcesMatrix(Point const &machinePos, float F[HANGPRINTER_MAX_ANCHORS]) const noexcept;
  void StaticForcesMatrixTetrahedron(Point const &machinePos, float F[HANGPRINTER_MAX_ANCHORS]) const noexcept;
  void StaticForcesMatrixQuadrilateralPyramid(Point const &machinePos, float F[HANGPRINTER_MAX_ANCHORS]) const noexcept;

public:
  Flex(size_t const numAnchors_) : Flex(numAnchors_, defaultMoverWeight_kg) {}

  explicit Flex(size_t const numAnchors_, float moverWeight_kg_) : numAnchors(numAnchors_), moverWeight_kg(moverWeight_kg_)  {
    for (size_t i = 0; i < numAnchors; ++i) {
      if (numAnchors == 3) {
        anchors[i] = threeAnchors[i];
      } else if (numAnchors == 4) {
        anchors[i] = fourAnchors[i];
      } else if (numAnchors == 5) {
        anchors[i] = fiveAnchors[i];
      } else {
        std::cout << "Non-supported number of anchors: " << numAnchors << '\n';
        return;
      }
      spoolRadii[i] = 75.0F;
      spoolRadiiSq[i] = spoolRadii[i] * spoolRadii[i];
      mechanicalAdvantage[i] = 2;
      fullStepsPerMotorRev[i] = 25;
      spoolGearTeeth[i] = 255;
      motorGearTeeth[i] = 20;
      linesPerSpool[i] = 1;
      minPlannedForce[i] = 0.0F;
      maxPlannedForce[i] = 120.0F;
    }

    targetForce_Newton = 20.0F;
    mechanicalAdvantage[numAnchors - 1] = 4; // top anchor has more mech adv

    Initk0k2();
    InitOrigin();
  }
  explicit Flex(std::array<Point, 5> const &anchors_) : Flex(anchors_.size()){
    for (size_t i{0}; i < anchors_.size(); ++i) {
      anchors[i] = anchors_[i];
    }
    InitOrigin();
  };

  explicit Flex(std::array<Point, 4> const &anchors_) : Flex(anchors_.size()){
    for (size_t i{0}; i < anchors_.size(); ++i) {
      anchors[i] = anchors_[i];
    }
    InitOrigin();
  };

  std::array<float, HANGPRINTER_MAX_ANCHORS> CartesianToMotorSteps(Point const machinePos, std::array<ssize_t, HANGPRINTER_MAX_ANCHORS> &motorPos, bool print = false) const noexcept;
  std::array<float, HANGPRINTER_MAX_ANCHORS> CartesianToMotorStepsMatrix(Point const machinePos, std::array<float, HANGPRINTER_MAX_ANCHORS> &motorPos) const noexcept;
  void PrintPretension() const;
  float MotorPosToLinePos(const float motorPos, size_t axis) const noexcept;
};

