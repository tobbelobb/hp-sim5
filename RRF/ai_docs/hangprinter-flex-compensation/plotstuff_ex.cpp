
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>

#include <flex_via_pseudoinverse.hpp>

enum class ConfigType { HP5 = 0, Slideprinter = 1, Spidercam = 2 };
enum class SolverType { TikhonovGD = 0, QP = 1 };
enum class Plane { XY = 0, XZ = 1, YZ = 2 };

static std::string to_lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c){ return std::tolower(c); });
  return s;
}

static void usage(const char* prog) {
  std::cerr <<
    "Usage: " << prog << " [-c|--config hp5|slideprinter|spidercam|0|1|2] "
    "[-s|--solver tikhonov_gradient_descent|qp|0|1] "
    "[-b|--both] [--xy|--xz|--yz]\n"
    "Defaults: --config hp5, --solver tikhonov_gradient_descent, --xy\n";
}

static bool parse_config_arg(const std::string& v, ConfigType& out) {
  std::string s = to_lower(v);
  if (s == "hp5" || s == "0") { out = ConfigType::HP5; return true; }
  if (s == "slideprinter" || s == "1") { out = ConfigType::Slideprinter; return true; }
  if (s == "spidercam" || s == "2") { out = ConfigType::Spidercam; return true; }
  return false;
}

static bool parse_solver_arg(const std::string& v, SolverType& out) {
  std::string s = to_lower(v);
  if (s == "thikonov_gradient_descent" || s == "tikhonov_gradient_descent" ||
      s == "thikonov" || s == "tikhonov" || s == "0") {
    out = SolverType::TikhonovGD; return true;
  }
  if (s == "qp" || s == "1") { out = SolverType::QP; return true; }
  return false;
}

static const char* solver_short(SolverType s) {
  return (s == SolverType::TikhonovGD) ? "Tikhonov+GD" : "QP";
}

static const char* plane_short(Plane p) {
  switch (p) {
    case Plane::XY: return "XY";
    case Plane::XZ: return "XZ";
    case Plane::YZ: return "YZ";
  }
  return "XY";
}

static char label_for(int idx) {
  return (idx == 4) ? 'I' : static_cast<char>('A' + idx);
}

struct GridMapper {
  Plane plane{Plane::XY};
  float x0{0.0f}, y0{0.0f}, z0{0.0f}; // fixed coords for the orthogonal axis

  // Map (a,b) -> (gridX, gridY) and mover Vec3
  inline void map(float a, float b, float& gridX, float& gridY, Vec3& mover) const {
    switch (plane) {
      case Plane::XY: gridX = a; gridY = b; mover = Vec3{a, b, z0}; break;
      case Plane::XZ: gridX = a; gridY = b; mover = Vec3{a, y0, b}; break;
      case Plane::YZ: gridX = a; gridY = b; mover = Vec3{x0, a, b}; break;
    }
  }
};

int main(int argc, char** argv) {
  ConfigType config = ConfigType::HP5;
  SolverType solver = SolverType::TikhonovGD;
  bool both = false;
  Plane plane = Plane::XY;

  // Parse CLI
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "-h" || a == "--help") {
      usage(argv[0]); return 0;
    } else if (a == "-c" || a == "--config") {
      if (i + 1 >= argc) { usage(argv[0]); return 1; }
      if (!parse_config_arg(argv[++i], config)) { usage(argv[0]); return 1; }
    } else if (a == "-s" || a == "--solver") {
      if (i + 1 >= argc) { usage(argv[0]); return 1; }
      if (!parse_solver_arg(argv[++i], solver)) { usage(argv[0]); return 1; }
    } else if (a == "-b" || a == "--both") {
      both = true;
    } else if (a == "--xy") {
      plane = Plane::XY;
    } else if (a == "--xz") {
      plane = Plane::XZ;
    } else if (a == "--yz") {
      plane = Plane::YZ;
    } else {
      std::cerr << "Unknown arg: " << a << "\n";
      usage(argv[0]);
      return 1;
    }
  }

  StaticForcesConfig cfg{};
  cfg.massKg = 20.0f;

  // Grid extents (simple symmetric defaults; adjust if you like)
  const float min = -1611.0f;
  const float max =  1601.0f;
  const float step = 35.0f;
  const float steps = std::floor((max - min) / step);

  // Anchors and bounds per config
  std::vector<Vec3> anchors;
  std::vector<float> Tmin, Tmax;
  {
    switch (config) {
      case ConfigType::HP5: {
        anchors = {
          Vec3{16.4f,     -1610.98f, -131.53f},
          Vec3{1314.22f,    128.14f, -121.28f},
          Vec3{-15.73f,    1415.61f, -121.82f},
          Vec3{-1211.62f,    18.14f, -111.18f},
          Vec3{10.0f,       -10.0f,  2299.83f}
        };
        cfg.ignorePretension = false;
        cfg.ignoreGravity = false;

        float tminI     = cfg.massKg * cfg.g;
        float tmaxI     = 240.0f;
        float tminABCD  = 20.0f;
        float tmaxABCD  = cfg.massKg * cfg.g;

        Tmin = {tminABCD, tminABCD, tminABCD, tminABCD, tminI};
        Tmax = {tmaxABCD, tmaxABCD, tmaxABCD, tmaxABCD, tmaxI};
        break;
      }
      case ConfigType::Slideprinter: {
        anchors = {
          Vec3{0.0f,       -1900.0f,   0.0f},
          Vec3{1645.448f,    949.99999f,0.0f},
          Vec3{-1645.448f,   949.99999f,0.0f}
        };
        cfg.ignorePretension = false;
        cfg.ignoreGravity = true;

        float tminABCD = 20.0f;
        float tmaxABCD = 120.0f;
        Tmin = {tminABCD, tminABCD, tminABCD};
        Tmax = {tmaxABCD, tmaxABCD, tmaxABCD};
        break;
      }
      case ConfigType::Spidercam: {
        anchors = {
          Vec3{16.4f,     -1610.98f, 1300.0f},
          Vec3{1314.22f,    128.14f, 1300.0f},
          Vec3{-15.73f,    1415.61f, 1300.0f},
          Vec3{-1211.62f,    18.14f, 1300.0f},
        };
        cfg.ignorePretension = true;
        cfg.ignoreGravity = false;

        float tminABCD = 10.0f;
        float tmaxABCD = cfg.massKg * cfg.g;
        Tmin = {tminABCD, tminABCD, tminABCD, tminABCD};
        Tmax = {tmaxABCD, tmaxABCD, tmaxABCD, tmaxABCD};
        break;
      }
    }
  }

  const int numAnchors = static_cast<int>(anchors.size());
  const int numPlots = numAnchors;

  cfg.Tmin = Tmin.data();
  cfg.Tmax = Tmax.data();

  // Mapper for plane selection. Fixed coordinate defaults to 0.
  GridMapper mapper;
  mapper.plane = plane;
  mapper.x0 = 0.0f; mapper.y0 = 0.0f; mapper.z0 = 0.0f;

  // Data containers
  std::vector<std::vector<float>> X;
  std::vector<std::vector<float>> Y;

  std::vector<std::vector<std::vector<float>>> Z(numPlots);
  std::vector<std::vector<float>> supportedFrac;
  std::vector<std::vector<float>> residualZ;

  // Both-mode containers
  std::vector<std::vector<std::vector<float>>> ZT(numPlots), ZQ(numPlots), ZD(numPlots);
  std::vector<std::vector<float>> fracT, fracQ, fracD;
  std::vector<std::vector<float>> resT,  resQ,  resD;

  auto reserve_gridlike = [&](auto& vv){
    vv.reserve(static_cast<size_t>(steps * steps));
  };

  X.reserve(static_cast<size_t>(steps * steps));
  Y.reserve(static_cast<size_t>(steps * steps));
  if (!both) {
    reserve_gridlike(supportedFrac);
    reserve_gridlike(residualZ);
    for (int i = 0; i < numPlots; ++i) reserve_gridlike(Z[i]);
  } else {
    reserve_gridlike(fracT); reserve_gridlike(fracQ); reserve_gridlike(fracD);
    reserve_gridlike(resT);  reserve_gridlike(resQ);  reserve_gridlike(resD);
    for (int i = 0; i < numPlots; ++i) {
      reserve_gridlike(ZT[i]); reserve_gridlike(ZQ[i]); reserve_gridlike(ZD[i]);
    }
  }

  // Results & tension buffers
  std::vector<float> tensions_t(numAnchors, 0.0f), tensions_q(numAnchors, 0.0f);
  StaticForcesResult R_t{}; R_t.tensions = tensions_t.data();
  StaticForcesResult R_q{}; R_q.tensions = tensions_q.data();

  // Sweep over the chosen plane: (a,b) map → (x,y,z)
  for (float b = min; b < max; b += step) {
    std::vector<float> xl;
    std::vector<float> yl;

    if (!both) {
      std::vector<std::vector<float>> zl_row(numPlots);
      std::vector<float> frac_row, res_row;

      for (float a = min; a < max; a += step) {
        float gx, gy; Vec3 mover;
        mapper.map(a, b, gx, gy, mover);
        xl.push_back(gx);
        yl.push_back(gy);

        if (solver == SolverType::TikhonovGD) {
          StaticForcesEx(anchors.data(), numAnchors, mover, cfg, R_t);
          for (int i = 0; i < numPlots; ++i) zl_row[i].push_back(tensions_t[i]);
          frac_row.push_back(R_t.supportedGravityFrac);
          res_row.push_back(R_t.residual.z);
        } else {
          StaticForcesEx_qp(anchors.data(), numAnchors, mover, cfg, R_q);
          for (int i = 0; i < numPlots; ++i) zl_row[i].push_back(tensions_q[i]);
          frac_row.push_back(R_q.supportedGravityFrac);
          res_row.push_back(R_q.residual.z);
        }
      }

      X.push_back(std::move(xl));
      Y.push_back(std::move(yl));
      supportedFrac.push_back(std::move(frac_row));
      residualZ.push_back(std::move(res_row));
      for (int i = 0; i < numPlots; ++i) Z[i].push_back(std::move(zl_row[i]));
    } else {
      std::vector<std::vector<float>> zt_row(numPlots), zq_row(numPlots), zd_row(numPlots);
      std::vector<float> fracT_row, fracQ_row, fracD_row;
      std::vector<float> resT_row,  resQ_row,  resD_row;

      for (float a = min; a < max; a += step) {
        float gx, gy; Vec3 mover;
        mapper.map(a, b, gx, gy, mover);
        xl.push_back(gx);
        yl.push_back(gy);

        StaticForcesEx(anchors.data(), numAnchors, mover, cfg, R_t);
        StaticForcesEx_qp(anchors.data(), numAnchors, mover, cfg, R_q);

        for (int i = 0; i < numPlots; ++i) {
          zt_row[i].push_back(tensions_t[i]);
          zq_row[i].push_back(tensions_q[i]);
          zd_row[i].push_back(tensions_q[i] - tensions_t[i]);
        }
        fracT_row.push_back(R_t.supportedGravityFrac);
        fracQ_row.push_back(R_q.supportedGravityFrac);
        fracD_row.push_back(R_q.supportedGravityFrac - R_t.supportedGravityFrac);

        resT_row.push_back(R_t.residual.z);
        resQ_row.push_back(R_q.residual.z);
        resD_row.push_back(R_q.residual.z - R_t.residual.z);
      }

      X.push_back(std::move(xl));
      Y.push_back(std::move(yl));

      fracT.push_back(std::move(fracT_row));
      fracQ.push_back(std::move(fracQ_row));
      fracD.push_back(std::move(fracD_row));
      resT.push_back(std::move(resT_row));
      resQ.push_back(std::move(resQ_row));
      resD.push_back(std::move(resD_row));

      for (int i = 0; i < numPlots; ++i) {
        ZT[i].push_back(std::move(zt_row[i]));
        ZQ[i].push_back(std::move(zq_row[i]));
        ZD[i].push_back(std::move(zd_row[i]));
      }
    }
  }

  // Python header
  std::cout << "from mpl_toolkits.mplot3d import axes3d\n";
  std::cout << "import matplotlib.pyplot as plt\n";
  std::cout << "from matplotlib import cm\n";
  std::cout << "import numpy as np\n\n";

  auto print_grid = [](auto const &grid, auto const &name) {
    std::cout << name << " = np.array([";
    std::string delim0;
    for (auto const &row : grid) {
      std::cout << delim0 << "[";
      delim0 = "],\n              ";
      std::string delim1;
      for (auto const &value : row) {
        std::cout << delim1 << value;
        delim1 = ", ";
      }
    }
    std::cout << "]])\n";
  };

  // Dump grids
  print_grid(X, "X");
  print_grid(Y, "Y");

  const char* planeTag = plane_short(plane);

  if (!both) {
    for (int i = 0; i < numPlots; ++i) print_grid(Z[i], "Z" + std::to_string(i));
    print_grid(supportedFrac, "Z_frac");
    print_grid(residualZ,    "Z_res");

    for (int i = 0; i < numPlots; ++i) {
      std::cout << "ax" << i
                << " = plt.figure().add_subplot(111, projection='3d', title='"
                << label_for(i) << " (" << solver_short(solver) << ", plane " << planeTag << ")')\n";
    }
    std::cout << "ax_frac = plt.figure().add_subplot(111, projection='3d', title='Supported gravity fraction ("
              << solver_short(solver) << ", plane " << planeTag << ")')\n";
    std::cout << "ax_res  = plt.figure().add_subplot(111, projection='3d', title='Gravity residual Z ("
              << solver_short(solver) << ", plane " << planeTag << ")')\n\n";

    for (int i = 0; i < numPlots; ++i) {
      std::cout << "ax" << i << ".plot_surface(X, Y, Z" << i << ", cmap=cm.viridis)\n";
    }
    std::cout << "ax_frac.plot_surface(X, Y, Z_frac, cmap=cm.coolwarm)\n";
    std::cout << "ax_res.plot_surface(X, Y, Z_res, cmap=cm.coolwarm)\n\n";
  } else {
    for (int i = 0; i < numPlots; ++i) {
      print_grid(ZT[i], "ZT" + std::to_string(i));
      print_grid(ZQ[i], "ZQ" + std::to_string(i));
      print_grid(ZD[i], "ZD" + std::to_string(i));
    }
    print_grid(fracT, "Z_frac_T");
    print_grid(fracQ, "Z_frac_Q");
    print_grid(fracD, "Z_frac_D");
    print_grid(resT,  "Z_res_T");
    print_grid(resQ,  "Z_res_Q");
    print_grid(resD,  "Z_res_D");

    for (int i = 0; i < numPlots; ++i) {
      char L = label_for(i);
      std::cout << "axT" << i << " = plt.figure().add_subplot(111, projection='3d', title='"
                << L << " (Tikhonov+GD, plane " << planeTag << ")')\n";
      std::cout << "axQ" << i << " = plt.figure().add_subplot(111, projection='3d', title='"
                << L << " (QP, plane " << planeTag << ")')\n";
      std::cout << "axD" << i << " = plt.figure().add_subplot(111, projection='3d', title='"
                << L << " (QP - Tikhonov, plane " << planeTag << ")')\n";
    }
    std::cout << "ax_frac_T = plt.figure().add_subplot(111, projection='3d', title='Supported gravity fraction (Tikhonov+GD, plane " << planeTag << ")')\n";
    std::cout << "ax_frac_Q = plt.figure().add_subplot(111, projection='3d', title='Supported gravity fraction (QP, plane " << planeTag << ")')\n";
    std::cout << "ax_frac_D = plt.figure().add_subplot(111, projection='3d', title='Supported gravity fraction (QP - Tikhonov, plane " << planeTag << ")')\n";
    std::cout << "ax_res_T  = plt.figure().add_subplot(111, projection='3d', title='Gravity residual Z (Tikhonov+GD, plane " << planeTag << ")')\n";
    std::cout << "ax_res_Q  = plt.figure().add_subplot(111, projection='3d', title='Gravity residual Z (QP, plane " << planeTag << ")')\n";
    std::cout << "ax_res_D  = plt.figure().add_subplot(111, projection='3d', title='Gravity residual Z (QP - Tikhonov, plane " << planeTag << ")')\n\n";

    for (int i = 0; i < numPlots; ++i) {
      std::cout << "axT" << i << ".plot_surface(X, Y, ZT" << i << ", cmap=cm.viridis)\n";
      std::cout << "axQ" << i << ".plot_surface(X, Y, ZQ" << i << ", cmap=cm.viridis)\n";
      std::cout << "axD" << i << ".plot_surface(X, Y, ZD" << i << ", cmap=cm.coolwarm)\n";
    }
    std::cout << "ax_frac_T.plot_surface(X, Y, Z_frac_T, cmap=cm.coolwarm)\n";
    std::cout << "ax_frac_Q.plot_surface(X, Y, Z_frac_Q, cmap=cm.coolwarm)\n";
    std::cout << "ax_frac_D.plot_surface(X, Y, Z_frac_D, cmap=cm.coolwarm)\n";
    std::cout << "ax_res_T.plot_surface(X, Y, Z_res_T, cmap=cm.coolwarm)\n";
    std::cout << "ax_res_Q.plot_surface(X, Y, Z_res_Q, cmap=cm.coolwarm)\n";
    std::cout << "ax_res_D.plot_surface(X, Y, Z_res_D, cmap=cm.coolwarm)\n\n";
  }

  std::cout << "plt.show()\n";
  return 0;
}
