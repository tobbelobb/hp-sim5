#include <array>
#include <cmath>
#include <iostream>
#include <vector>

#include <Matrix.h>
#include <flex.hpp>
#include <util.hpp>

// Requires numAnchors
void Flex::Initk0k2() {
  float stepsPerUnitTimesRTmp[HANGPRINTER_MAX_ANCHORS] = {0.0};
  for (size_t i = 0; i < numAnchors; i++) {
    stepsPerUnitTimesRTmp[i] =
        ((float)(mechanicalAdvantage[i]) * fullStepsPerMotorRev[i] * microsteps * spoolGearTeeth[i]) / (2.0 * Pi * motorGearTeeth[i]);

    k2[i] = -(float)(mechanicalAdvantage[i] * linesPerSpool[i]) * spoolBuildupFactor;
    k0[i] = 2.0 * stepsPerUnitTimesRTmp[i] / k2[i];
  }
}

// Requires numAnchors and anchors
void Flex::InitOrigin() {
  for (size_t i = 0; i < numAnchors; ++i) {
    distancesOrigin[i] = sqrtf(fsquare(anchors[i][0]) + fsquare(anchors[i][1]) + fsquare(anchors[i][2]));
  }
  for (size_t i{0}; i < numAnchors - 1; ++i) {
    guyWireLengths[i] = hyp3(anchors[i], anchors[numAnchors - 1]);
  }
  guyWireLengths[numAnchors - 1] = 0.0F;
  for (size_t i{0}; i < numAnchors; ++i) {
    springKsOrigin[i] = SpringK(distancesOrigin[i] * mechanicalAdvantage[i] + guyWireLengths[i]);
  }
  StaticForcesMatrix({0.0, 0.0, 0.0}, fOriginMatrix);
  for (int i{0}; i < numAnchors; ++i) {
    relaxedSpringLengthsOrigin[i] = distancesOrigin[i] - fOriginMatrix[i] / (springKsOrigin[i] * mechanicalAdvantage[i]);
  }
  for (int i{0}; i < numAnchors; ++i) {
    relaxedSpringLengthsOriginMatrix[i] = distancesOrigin[i] - fOriginMatrix[i] / (springKsOrigin[i] * mechanicalAdvantage[i]);
  }
}

float Flex::SpringK(float const springLength) const noexcept {
  // std::cout << "Made a new springK:\n" << springKPerUnitLength / springLength << '\n';
  return springKPerUnitLength / springLength;
}

void Flex::StaticForcesMatrix(Point const &machinePos, float F[HANGPRINTER_MAX_ANCHORS]) const noexcept {
  if (numAnchors == 4) {
    StaticForcesMatrixTetrahedron(machinePos, F);
  } else if (numAnchors == 5) {
    StaticForcesMatrixQuadrilateralPyramid(machinePos, F);
  }
}

void Flex::StaticForcesMatrixQuadrilateralPyramid(Point const &machinePos, float F[HANGPRINTER_MAX_ANCHORS]) const noexcept {
  // A QuadrilateralPyramid has 5 corners, there's one anchor in each.
  // There are many 4's in this function because 4 motors (the lower ones, ABCD)
  // are assumed to have unknown forces.
  // The forces in the top anchor is assumed to be known and constant, except for gravity's
  // effects who are also known.
  if (moverWeight_kg < 0.0001) {
    return;
  }
  // Space for four linear 3x3 systems, each with two solution columns,
  FixedMatrix<float, 3, 5> M[4];

  float norm[5];
  norm[4] = hyp3(anchors[4], machinePos);
  for (int i = 0; i < 4; ++i) {
    norm[i] = hyp3(anchors[i], machinePos);
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 4; ++k) {
        // Fill 3x3 top left corner of system with
        // unit vectors toward each ABCD anchor from mover
        // If A is the column vector pointing towards A-anchor, we're building these
        // four matrices:
        // k=0: [BCD], A-direction skipped
        // k=1: [ACD], B-direction skipped
        // k=2: [ABD], C-direction skipped
        // k=3: [ABC], D-direction skipped
        if ( k != i) {
          if ( i > k ) {
            M[k](j, i - 1) = (anchors[i][j] - machinePos[j]) / norm[i];
          } else {
            M[k](j, i) = (anchors[i][j] - machinePos[j]) / norm[i];
          }
        }
      }
    }
  }
  float const mg = moverWeight_kg * 9.81;

  float top_mg = 0.0F;
  float top_pre = 0.0F;

  if (anchors[4][Z_AXIS] > machinePos[Z_AXIS]) {
    // These force constants will go into the solution column that has to do with gravity
    top_mg = mg / ((anchors[4][Z_AXIS] - machinePos[Z_AXIS]) / norm[4]);
    top_pre = targetForce_Newton;
  }

  // Indices for the two solution columns
  size_t const sol_mg = 3;
  size_t const sol_pt = 4;
  for (int i = 0; i < 3; ++i) {
    float const top_dist = (anchors[4][i] - machinePos[i]) / norm[4];
    for (int k = 0; k < 4; ++k) {
      M[k](i, sol_mg) = -top_mg * top_dist;  // gravity solution column
      M[k](i, sol_pt) = -top_pre * top_dist; // pretension solution column
    }
  }
  for (int k = 0; k < 4; ++k) {
    // Cancel out top anchor's Z-force with gravity.
    M[k](Z_AXIS, sol_mg) += mg; // == 0
  }

  // Solve the four systems
  for (int k = 0; k < 4; ++k) {
    M[k].GaussJordan(3, 5);
  }

  // Weigh/scale the pre-tension solutions so all have equal max force.
  float norm_ABCD[4];
  for(size_t k{0}; k < 4; ++k) {
    norm_ABCD[k] = fastSqrtf(M[k](0, sol_pt) * M[k](0, sol_pt) + M[k](1, sol_pt) * M[k](1, sol_pt) + M[k](2, sol_pt) * M[k](2, sol_pt));
  }

  // Arrays to hold our weighted combinations of the four (pairs of) solutions
  float p[4] = { 0.0F, 0.0F, 0.0F, 0.0F };
  float m[4] = { 0.0F, 0.0F, 0.0F, 0.0F };
  for (size_t i{0}; i < 3; ++i) {
    for (size_t j{0}; j < 4; ++j) {
      float const pt_weight = targetForce_Newton / norm_ABCD[j];
      // The gravity counter actions are scaled to exactly counter act gravity, and top-line forces neccesary to counter act gravity.
      // So the resultant force of all four solutions is the same. Lets add a quarter of each solution to get back that resultant force.
      float const mg_weight = 1.0/4.0;
      // i can mean BCD, ACD, ABD, or ABC, depending on which matrix we're looking into
      // Let's just translate that back into the solutions vectors
      size_t const s = j <= i ? i + 1 : i;
      p[s] += M[j](i, sol_pt)*pt_weight;
      m[s] += M[j](i, sol_mg)*mg_weight;
    }
  }

  // The pre-tension solution can be scaled up or down however we want.
  // Forces in those solution cancel each other out exactly, so any multiple of the solution is also a valid solution.
  //
  // (The gravity solution can't be scaled since it has to exactly counter act top-line forces that must exactly counter act gravity (mg))
  //
  // Use the scaling freedom of the pre-tension solution to assure that we have at least targetForce_Newton in the ABCD lines,
  // and that no line (incl top-line) get more tension than the configured maxPlannedForce in that direction.
  float  preFac = min(max(std::abs((targetForce_Newton - m[3]) / p[3]),
                               max(std::abs((targetForce_Newton - m[2]) / p[2]),
                                   max(std::abs((targetForce_Newton - m[1]) / p[1]), std::abs((targetForce_Newton - m[0]) / p[0])))),
                           min(std::abs((maxPlannedForce[4] - top_mg) / top_pre),
                               min(min(std::abs((maxPlannedForce[0] - m[0]) / p[0]), std::abs((maxPlannedForce[1] - m[1]) / p[1])),
                                   min(std::abs((maxPlannedForce[2] - m[2]) / p[2]), std::abs((maxPlannedForce[3] - m[3]) / p[3])))));

  float tot[5] = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
  tot[0] = m[0] + preFac * p[0];
  tot[1] = m[1] + preFac * p[1];
  tot[2] = m[2] + preFac * p[2];
  tot[3] = m[3] + preFac * p[3];
  tot[4] = top_mg + preFac * top_pre;

  for (size_t i{0}; i < 5; ++i) {
    // Negative, or very large forces can still have slipped through the preFac filter.
    // Truncate away such forces and assign to the output variable.
    // Voila.
    // The min( ... ) shouldn't be needed here. Just better safe than sorry.
    F[i] = min(max(tot[i], minPlannedForce[i]), maxPlannedForce[i]);
  }
}

void Flex::StaticForcesMatrixTetrahedron(Point const &machinePos, float F[HANGPRINTER_MAX_ANCHORS]) const noexcept {
  static constexpr size_t A_AXIS = 0;
  static constexpr size_t B_AXIS = 1;
  static constexpr size_t C_AXIS = 2;
  static constexpr size_t D_AXIS = 3;
  static constexpr size_t OLD_DEFAULT_NUM_ANCHORS = 4;
  static constexpr size_t CARTESIAN_AXES = 3;

  if (moverWeight_kg > 0.0001) { // mover weight more than one gram
    float norm[OLD_DEFAULT_NUM_ANCHORS]; // Unit vector directions toward each anchor from mover
    FixedMatrix<float, CARTESIAN_AXES, OLD_DEFAULT_NUM_ANCHORS + 1> M;
    for (size_t i = 0; i < OLD_DEFAULT_NUM_ANCHORS - 1; ++i) { // One anchor above mover
      norm[i] = hyp3(anchors[i], machinePos);
      for (size_t j = 0; j < CARTESIAN_AXES; ++j) {
        M(j, i) = (anchors[i][j] - machinePos[j]) / norm[i];
      }
    }

    float const mg = moverWeight_kg * 9.81; // Size of gravity force in Newtons
    float D_mg = 0.0F;
    float D_pre = 0.0F;

    // The D-forces' z-component is always equal to mg + targetForce_Newton.
    // The D-forces' z-component is always equal to mg +
    // targetForce_Newton. This means ABC-motors combined pull
    // downwards targetForce_Newton N. I don't know if that's always
    // solvable. Still, my tests show that we get very reasonable
    // flex compensation...

    // Right hand side of the equation
    // A + B + C + D + (0,0,-mg)' = 0
    // <=> A + B + C = -D + (0,0,mg)'
    //
    // Mx = y,
    //
    // Where M is the matrix
    //
    //     ax bx cx
    // M = ay by cy,
    //     az bz cz
    //
    // and x is the sizes of the forces:
    //
    //     A
    // x = B,
    //     C
    //
    // and y is
    //
    //     -D*dx
    // y = -D*dy     .
    //     -D*dz + mg

    float const normD = hyp3(anchors[D_AXIS], machinePos);
    if (anchors[D_AXIS][Z_AXIS] > machinePos[Z_AXIS]) { // D anchor above machine
      D_mg = mg / ((anchors[D_AXIS][2] - machinePos[2]) / normD);
      D_pre = targetForce_Newton;
    }

    for (size_t i = 0; i < CARTESIAN_AXES; ++i) {
      float const dist = (anchors[D_AXIS][i] - machinePos[i]) / normD;
      M(i, OLD_DEFAULT_NUM_ANCHORS - 1) = -D_mg * dist;
      M(i, OLD_DEFAULT_NUM_ANCHORS) = -D_pre * dist;
    }
    M(Z_AXIS, D_AXIS) += mg;

    // Solve!
    const bool ok = M.GaussJordan(CARTESIAN_AXES, 5);

    if (ok) {
      // Size of the undetermined forces
      float const A_mg = M(0, 3);
      float const B_mg = M(1, 3);
      float const C_mg = M(2, 3);
      float const A_pre = M(0, 4);
      float const B_pre = M(1, 4);
      float const C_pre = M(2, 4);

      // Assure at least targetForce in the ABC lines (first argument to outer min()),
      // and that no line get more than max planned force (second argument to outer min()).
      float const preFac = min(max(std::abs((targetForce_Newton - C_mg) / C_pre),
                                   max(std::abs((targetForce_Newton - B_mg) / B_pre), std::abs((targetForce_Newton - A_mg) / A_pre))),
                               min(min(std::abs((maxPlannedForce[A_AXIS] - A_mg) / A_pre), std::abs((maxPlannedForce[B_AXIS] - B_mg) / B_pre)),
                                   min(std::abs((maxPlannedForce[C_AXIS] - C_mg) / C_pre), std::abs((maxPlannedForce[D_AXIS] - D_mg) / D_pre))));

      float totalForces[OLD_DEFAULT_NUM_ANCHORS] = {
        A_mg + preFac * A_pre,
        B_mg + preFac * B_pre,
        C_mg + preFac * C_pre,
        D_mg + preFac * D_pre
      };

      for (size_t i = 0; i < OLD_DEFAULT_NUM_ANCHORS; ++i) {
        F[i] = min(max(totalForces[i], minPlannedForce[i]), maxPlannedForce[i]);
      }
    }
  }
}

float Flex::MotorPosToLinePos(const float motorPos, size_t axis) const noexcept {
  return (fsquare(motorPos / k0[axis] + spoolRadii[axis]) - spoolRadiiSq[axis]) / k2[axis];
}

// Calculate the square of the line length from a spool from a Cartesian
// coordinate
inline float distancesSquared(Point const machinePos, std::array<float, 3> anchorPos) {
  return fsquare(anchorPos[Z_AXIS] - machinePos[Z_AXIS]) + fsquare(anchorPos[Y_AXIS] - machinePos[Y_AXIS]) +
         fsquare(anchorPos[X_AXIS] - machinePos[X_AXIS]);
}

void Flex::PrintPretension() const {
  std::array<float, HANGPRINTER_MAX_ANCHORS> pretension;
  for (size_t i = 0; i < numAnchors; ++i) {
    pretension[i] = relaxedSpringLengthsOrigin[i] - distancesOrigin[i];
  }
  std::cerr << "PrintPretension(): pretension:\n[ ";
  for (size_t i{0}; i < numAnchors; ++i) {
    if (i != 0) {
      std::cerr << ", ";
    }
    std::cerr << pretension[i];
  }
  std::cerr << " ]\n";
  std::cerr << "PrintPretension(): fOriginMatrix:\n[ ";
  for (size_t i{0}; i < numAnchors; ++i) {
    if (i != 0) {
      std::cerr << ", ";
    }
    std::cerr << fOriginMatrix[i];
  }
  std::cerr << " ]\n";
}

// Convert Cartesian coordinates to motor coordinates
std::array<float, HANGPRINTER_MAX_ANCHORS>
Flex::CartesianToMotorStepsMatrix(Point const machinePos, std::array<float, HANGPRINTER_MAX_ANCHORS> &motorPos) const noexcept {
  float squaredDistances[numAnchors];
  for (size_t i = 0; i < numAnchors; ++i) {
    squaredDistances[i] = distancesSquared(machinePos, anchors[i]);
  }

  float distances[numAnchors];
  for (int i{0}; i < numAnchors; ++i) {
    distances[i] = fastSqrtf(squaredDistances[i]);
  }

  float distanceDifferences[numAnchors];
  for (size_t i = 0; i < numAnchors; ++i) {
    distanceDifferences[i] = distances[i] - distancesOrigin[i];
  }

  // Similar calculation as distances, but taking line flex into account as described here:
  // https://torbjornludvigsen.com/blog/2018/#hangprinter_project_54

  float springKs[numAnchors];
  for (int i{0}; i < numAnchors; ++i) {
    springKs[i] = SpringK(distances[i] * mechanicalAdvantage[i] + guyWireLengths[i]);
  }

  float FMatrix[numAnchors] = {0.0F};
  StaticForcesMatrix(machinePos, FMatrix);
  // F is the desired force in each direction
  // F is not the tension in the line, because of mechanical advantage.

  // bool wrong = false;
  // for (int i = 0; i < numAnchors; ++i) {
  //  if (not (fabs(F[i] - FMatrix[i]) < 0.01)) {
  //    wrong = true;
  //  }
  //}
  // if (wrong) {
  //    std::cout << "Found something wrong\n";
  //    for (int i = 0; i < numAnchors; ++i) {
  //    std::cout << F[i] << ", ";
  //    }
  //  std::cout << '\n';
  //    for (int i = 0; i < numAnchors; ++i) {
  //    std::cout << FMatrix[i] << ", ";
  //    }
  //  std::cout << '\n';
  //}

  float relaxedSpringLengths[numAnchors];
  for (int i{0}; i < numAnchors; ++i) {
    relaxedSpringLengths[i] = distances[i] - FMatrix[i] / (springKs[i] * mechanicalAdvantage[i]);
    // The second term there is the mover's movement in mm due to flex
  }

  float linePos[numAnchors];
  for (size_t i = 0; i < numAnchors; ++i) {
    linePos[i] = relaxedSpringLengths[i] - relaxedSpringLengthsOriginMatrix[i];
  }
  // std::cout << "linePos: " << '[' << linePos[0] << ", " << linePos[1] << ", " << linePos[2] << ", " << linePos[3] << "]\n";

  std::array<float, HANGPRINTER_MAX_ANCHORS> impactOfSpringModel{};
  for (size_t i = 0; i < numAnchors; ++i) {
    impactOfSpringModel[i] = linePos[i] - distanceDifferences[i];
  }
  // std::cout << machinePos;
  // std::cout << " [" << std::setw(8) << std::fixed << std::setprecision(4) << std::right << impactOfSpringModel[0] << ", "
  //                  << std::setw(8) <<                                                     impactOfSpringModel[1] << ", "
  //                  << std::setw(8) <<                                                     impactOfSpringModel[2] << ", "
  //                  << std::setw(8) <<                                                     impactOfSpringModel[3] << std::defaultfloat <<
  //                  "]\n";

  for (size_t i{0}; i < numAnchors; ++i) {
    motorPos[i] = lrintf(k0[i] * (fastSqrtf(spoolRadiiSq[i] + linePos[i] * k2[i]) - spoolRadii[i]));
  }

  impactOfSpringModel[0] = FMatrix[0];
  impactOfSpringModel[1] = FMatrix[1];
  impactOfSpringModel[2] = FMatrix[2];
  impactOfSpringModel[3] = FMatrix[3];
  impactOfSpringModel[4] = FMatrix[4];
  return impactOfSpringModel;
}

std::array<float, HANGPRINTER_MAX_ANCHORS>
Flex::CartesianToMotorSteps(Point const machinePos, std::array<ssize_t, HANGPRINTER_MAX_ANCHORS> &motorPos, bool print) const noexcept {
  float squaredDistances[numAnchors];
  for (size_t i = 0; i < numAnchors; ++i) {
    squaredDistances[i] = distancesSquared(machinePos, anchors[i]);
  }

  float distances[numAnchors];
  for (int i{0}; i < numAnchors; ++i) {
    distances[i] = fastSqrtf(squaredDistances[i]);
  }

  float distanceDifferences[numAnchors];
  for (size_t i = 0; i < numAnchors; ++i) {
    distanceDifferences[i] = distances[i] - distancesOrigin[i];
  }

  // Similar calculation as distances, but taking line flex into account as described here:
  // https://torbjornludvigsen.com/blog/2018/#hangprinter_project_54

  float springKs[numAnchors];
  for (int i{0}; i < numAnchors; ++i) {
    springKs[i] = SpringK(distances[i] * mechanicalAdvantage[i] + guyWireLengths[i]);
  }

  float F[numAnchors] = {0.0F};
  StaticForcesMatrix(machinePos, F);
  // F is the desired force in each direction
  // F is not the tension in the line, because of mechanical advantage.

  float relaxedSpringLengths[numAnchors];
  for (int i{0}; i < numAnchors; ++i) {
    relaxedSpringLengths[i] = distances[i] - F[i] / (springKs[i] * mechanicalAdvantage[i]);
    // The second term there is the mover's movement in mm due to flex
  }

  float linePos[numAnchors];
  for (size_t i = 0; i < numAnchors; ++i) {
    linePos[i] = relaxedSpringLengths[i] - relaxedSpringLengthsOrigin[i];
  }
  // std::cout << "linePos: " << '[' << linePos[0] << ", " << linePos[1] << ", " << linePos[2] << ", " << linePos[3] << "]\n";

  std::array<float, HANGPRINTER_MAX_ANCHORS> impactOfSpringModel{};
  for (size_t i = 0; i < numAnchors; ++i) {
    impactOfSpringModel[i] = linePos[i] - distanceDifferences[i];
  }
  // std::cout << machinePos;
  // std::cout << " [" << std::setw(8) << std::fixed << std::setprecision(4) << std::right << impactOfSpringModel[0] << ", "
  //                  << std::setw(8) <<                                                     impactOfSpringModel[1] << ", "
  //                  << std::setw(8) <<                                                     impactOfSpringModel[2] << ", "
  //                  << std::setw(8) <<                                                     impactOfSpringModel[3] << std::defaultfloat <<
  //                  "]\n";

  for (size_t i{0}; i < numAnchors; ++i) {
    motorPos[i] = lrintf(k0[i] * (fastSqrtf(spoolRadiiSq[i] + linePos[i] * k2[i]) - spoolRadii[i]));
  }

  return impactOfSpringModel;
}
