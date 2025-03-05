#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

/// This file contains a simple FLIP fluid simulation implementation. It is
/// based on the presentation / video by Matthias Müller-Fischer:
/// - https://youtu.be/XmzBREkK8kY
/// - https://github.com/matthias-research/pages/blob/master/tenMinutePhysics/18-flip.html
///
/// The implementation is not optimized for performance, but for simplicity and
/// readability. It is not meant to be used in production code, but as a
/// reference for understanding the basics of FLIP simulations.
namespace fluid {
enum CellType {
  FLUID,
  AIR,
  SOLID,
}; // enum class CellType

/// C++ version of the above javascript code:
class FlipFluid {
public:
  FlipFluid(float density, int width, int height, float spacing, float particleRadius,
            int maxParticles)
      : density(density)
      , fNumX(std::floor(width / spacing) + 1)
      , fNumY(std::floor(height / spacing) + 1)
      , h(std::max(width / fNumX, height / fNumY))
      , fInvSpacing(1.0f / h)
      , fNumCells(fNumX * fNumY)
      , u(fNumCells)
      , v(fNumCells)
      , du(fNumCells)
      , dv(fNumCells)
      , prevU(fNumCells)
      , prevV(fNumCells)
      , p(fNumCells)
      , s(fNumCells)
      , cellType(fNumCells)
      , cellColor(3 * fNumCells)
      , maxParticles(maxParticles)
      , particlePos(2 * maxParticles)
      , particleColor(3 * maxParticles)
      , particleVel(2 * maxParticles)
      , particleDensity(fNumCells)
      , particleRestDensity(0.0f)
      , particleRadius(particleRadius)
      , pInvSpacing(1.0f / (2.2f * particleRadius))
      , pNumX(std::floor(width * pInvSpacing) + 1)
      , pNumY(std::floor(height * pInvSpacing) + 1)
      , pNumCells(pNumX * pNumY)
      , numCellParticles(pNumCells)
      , firstCellParticle(pNumCells + 1)
      , cellParticleIds(maxParticles)
      , numParticles(0) {}

  void integrateParticles(float dt, float gravity) {
    for (int i = 0; i < numParticles; i++) {
      particleVel[2 * i + 1] += dt * gravity;
      particlePos[2 * i] += particleVel[2 * i] * dt;
      particlePos[2 * i + 1] += particleVel[2 * i + 1] * dt;
    }
  }

  void pushParticlesApart(int numIters) {
    float colorDiffusionCoeff = 0.001f;

    // count particles per cell
    std::fill(numCellParticles.begin(), numCellParticles.end(), 0);

    for (int i = 0; i < numParticles; i++) {
      float x = particlePos[2 * i];
      float y = particlePos[2 * i + 1];

      int xi = std::clamp(static_cast<int>(x * pInvSpacing), 0, pNumX - 1);
      int yi = std::clamp(static_cast<int>(y * pInvSpacing), 0, pNumY - 1);
      int cellNr = xi * pNumY + yi;
      numCellParticles[cellNr]++;
    }

    // partial sums
    int first = 0;
    for (int i = 0; i < pNumCells; i++) {
      first += numCellParticles[i];
      firstCellParticle[i] = first;
    }
    firstCellParticle[pNumCells] = first; // guard

    // fill particles into cells
    for (int i = 0; i < numParticles; i++) {
      float x = particlePos[2 * i];
      float y = particlePos[2 * i + 1];

      int xi = std::clamp(static_cast<int>(x * pInvSpacing), 0, pNumX - 1);
      int yi = std::clamp(static_cast<int>(y * pInvSpacing), 0, pNumY - 1);
      int cellNr = xi * pNumY + yi;
      firstCellParticle[cellNr]--;
      cellParticleIds[firstCellParticle[cellNr]] = i;
    }

    // push particles apart
    float minDist = 2.0f * particleRadius;
    float minDist2 = minDist * minDist;

    for (int iter = 0; iter < numIters; iter++) {
      for (int i = 0; i < numParticles; i++) {
        float px = particlePos[2 * i];
        float py = particlePos[2 * i + 1];

        int pxi = static_cast<int>(px * pInvSpacing);
        int pyi = static_cast<int>(py * pInvSpacing);
        int x0 = std::max(pxi - 1, 0);
        int y0 = std::max(pyi - 1, 0);
        int x1 = std::min(pxi + 1, pNumX - 1);
        int y1 = std::min(pyi + 1, pNumY - 1);

        for (int xi = x0; xi <= x1; xi++) {
          for (int yi = y0; yi <= y1; yi++) {
            int cellNr = xi * pNumY + yi;
            int first = firstCellParticle[cellNr];
            int last = firstCellParticle[cellNr + 1];
            for (int j = first; j < last; j++) {
              int id = cellParticleIds[j];
              if (id == i)
                continue;
              float qx = particlePos[2 * id];
              float qy = particlePos[2 * id + 1];

              float dx = qx - px;
              float dy = qy - py;
              float d2 = dx * dx + dy * dy;
              if (d2 > minDist2 || d2 == 0.0f)
                continue;
              float d = std::sqrt(d2);
              float s = 0.5f * (minDist - d) / d;
              dx *= s;
              dy *= s;
              particlePos[2 * i] -= dx;
              particlePos[2 * i + 1] -= dy;
              particlePos[2 * id] += dx;
              particlePos[2 * id + 1] += dy;

              // diffuse colors
              for (int k = 0; k < 3; k++) {
                float color0 = particleColor[3 * i + k];
                float color1 = particleColor[3 * id + k];
                float color = (color0 + color1) * 0.5f;
                particleColor[3 * i + k] = color0 + (color - color0) * colorDiffusionCoeff;
                particleColor[3 * id + k] = color1 + (color - color1) * colorDiffusionCoeff;
              }
            }
          }
        }
      }
    }
  }

  void handleParticleCollisions(float obstacleX, float obstacleY, float obstacleRadius) {
    float h = 1.0f / fInvSpacing;
    float r = particleRadius;
    float or_ = obstacleRadius;
    // float or2 = or_ * or_;
    float minDist = obstacleRadius + r;
    float minDist2 = minDist * minDist;

    float minX = h + r;
    float maxX = (fNumX - 1) * h - r;
    float minY = h + r;
    float maxY = (fNumY - 1) * h - r;

    for (int i = 0; i < numParticles; i++) {
      float x = particlePos[2 * i];
      float y = particlePos[2 * i + 1];

      float dx = x - obstacleX;
      float dy = y - obstacleY;
      float d2 = dx * dx + dy * dy;

      // obstacle collision
      if (d2 < minDist2) {
        // TODO: obstacle collision velocity response
        // particleVel[2 * i] = scene.obstacleVelX;
        // particleVel[2 * i + 1] = scene.obstacleVelY;
      }

      // wall collisions
      if (x < minX) {
        x = minX;
        particleVel[2 * i] = 0.0f;
      }
      if (x > maxX) {
        x = maxX;
        particleVel[2 * i] = 0.0f;
      }
      if (y < minY) {
        y = minY;
        particleVel[2 * i + 1] = 0.0f;
      }
      if (y > maxY) {
        y = maxY;
        particleVel[2 * i + 1] = 0.0f;
      }
      particlePos[2 * i] = x;
      particlePos[2 * i + 1] = y;
    }
  }

  void updateParticleDensity() {
    float n = fNumY;
    float h = this->h;
    float h1 = fInvSpacing;
    float h2 = 0.5f * h;

    std::fill(particleDensity.begin(), particleDensity.end(), 0.0f);

    for (int i = 0; i < numParticles; i++) {
      float x = particlePos[2 * i];
      float y = particlePos[2 * i + 1];

      x = std::clamp(x, h, (fNumX - 1) * h);
      y = std::clamp(y, h, (fNumY - 1) * h);

      int x0 = std::floor((x - h2) * h1);
      float tx = ((x - h2) - x0 * h) * h1;
      int x1 = std::min(x0 + 1, fNumX - 2);

      int y0 = std::floor((y - h2) * h1);
      float ty = ((y - h2) - y0 * h) * h1;
      int y1 = std::min(y0 + 1, fNumY - 2);

      float sx = 1.0f - tx;
      float sy = 1.0f - ty;

      if (x0 < fNumX && y0 < fNumY)
        particleDensity[x0 * n + y0] += sx * sy;
      if (x1 < fNumX && y0 < fNumY)
        particleDensity[x1 * n + y0] += tx * sy;
      if (x1 < fNumX && y1 < fNumY)
        particleDensity[x1 * n + y1] += tx * ty;
      if (x0 < fNumX && y1 < fNumY)
        particleDensity[x0 * n + y1] += sx * ty;
    }

    if (particleRestDensity == 0.0f) {
      float sum = 0.0f;
      int numFluidCells = 0;

      for (int i = 0; i < fNumCells; i++) {
        if (cellType[i] == CellType::FLUID) {
          sum += particleDensity[i];
          numFluidCells++;
        }
      }

      if (numFluidCells > 0)
        particleRestDensity = sum / numFluidCells;
    }
  }

  void transferVelocities(bool toGrid, float flipRatio) {
    float n = fNumY;
    float h = this->h;
    float h1 = fInvSpacing;
    float h2 = 0.5f * h;

    if (toGrid) {
      prevU = u;
      prevV = v;

      std::fill(du.begin(), du.end(), 0.0f);
      std::fill(dv.begin(), dv.end(), 0.0f);
      std::fill(u.begin(), u.end(), 0.0f);
      std::fill(v.begin(), v.end(), 0.0f);

      for (int i = 0; i < fNumCells; i++)
        cellType[i] = s[i] == 0.0f ? CellType::SOLID : CellType::AIR;

      for (int i = 0; i < numParticles; i++) {
        float x = particlePos[2 * i];
        float y = particlePos[2 * i + 1];
        int xi = std::clamp(static_cast<int>(x * h1), 0, fNumX - 1);
        int yi = std::clamp(static_cast<int>(y * h1), 0, fNumY - 1);
        int cellNr = xi * n + yi;
        if (cellType[cellNr] == CellType::AIR)
          cellType[cellNr] = CellType::FLUID;
      }
    }

    for (int component = 0; component < 2; component++) {
      float dx = component == 0 ? 0.0f : h2;
      float dy = component == 0 ? h2 : 0.0f;

      auto &f = component == 0 ? u : v;
      auto &prevF = component == 0 ? prevU : prevV;
      auto &d = component == 0 ? du : dv;

      for (int i = 0; i < numParticles; i++) {
        float x = particlePos[2 * i];
        float y = particlePos[2 * i + 1];

        x = std::clamp(x, h, (fNumX - 1) * h);
        y = std::clamp(y, h, (fNumY - 1) * h);

        int x0 = std::min(static_cast<int>((x - dx) * h1), fNumX - 2);
        float tx = ((x - dx) - x0 * h) * h1;
        int x1 = std::min(x0 + 1, fNumX - 2);

        int y0 = std::min(static_cast<int>((y - dy) * h1), fNumY - 2);
        float ty = ((y - dy) - y0 * h) * h1;
        int y1 = std::min(y0 + 1, fNumY - 2);

        float sx = 1.0f - tx;
        float sy = 1.0f - ty;

        float d0 = sx * sy;
        float d1 = tx * sy;
        float d2 = tx * ty;
        float d3 = sx * ty;

        int nr0 = x0 * n + y0;
        int nr1 = x1 * n + y0;
        int nr2 = x1 * n + y1;
        int nr3 = x0 * n + y1;

        if (toGrid) {
          float pv = particleVel[2 * i + component];
          f[nr0] += pv * d0;
          d[nr0] += d0;
          f[nr1] += pv * d1;
          d[nr1] += d1;
          f[nr2] += pv * d2;
          d[nr2] += d2;
          f[nr3] += pv * d3;
          d[nr3] += d3;
        } else {
          int offset = component == 0 ? n : 1;
          float valid0 = cellType[nr0] != CellType::AIR || cellType[nr0 - offset] != CellType::AIR
                             ? 1.0f
                             : 0.0f;
          float valid1 = cellType[nr1] != CellType::AIR || cellType[nr1 - offset] != CellType::AIR
                             ? 1.0f
                             : 0.0f;
          float valid2 = cellType[nr2] != CellType::AIR || cellType[nr2 - offset] != CellType::AIR
                             ? 1.0f
                             : 0.0f;
          float valid3 = cellType[nr3] != CellType::AIR || cellType[nr3 - offset] != CellType::AIR
                             ? 1.0f
                             : 0.0f;

          float v = particleVel[2 * i + component];
          float d = valid0 * d0 + valid1 * d1 + valid2 * d2 + valid3 * d3;

          if (d > 0.0f) {
            float picV = (valid0 * d0 * f[nr0] + valid1 * d1 * f[nr1] + valid2 * d2 * f[nr2] +
                          valid3 * d3 * f[nr3]) /
                         d;
            float corr =
                (valid0 * d0 * (f[nr0] - prevF[nr0]) + valid1 * d1 * (f[nr1] - prevF[nr1]) +
                 valid2 * d2 * (f[nr2] - prevF[nr2]) + valid3 * d3 * (f[nr3] - prevF[nr3])) /
                d;
            float flipV = v + corr;

            particleVel[2 * i + component] = (1.0f - flipRatio) * picV + flipRatio * flipV;
          }
        }
      }

      if (toGrid) {
        for (int i = 0; i < f.size(); i++) {
          if (d[i] > 0.0f)
            f[i] /= d[i];
        }

        // restore solid cells
        for (int i = 0; i < fNumX; i++) {
          for (int j = 0; j < fNumY; j++) {
            bool solid = cellType[i * n + j] == CellType::SOLID;
            if (solid || (i > 0 && cellType[(i - 1) * n + j] == CellType::SOLID))
              u[i * n + j] = prevU[i * n + j];
            if (solid || (j > 0 && cellType[i * n + j - 1] == CellType::SOLID))
              v[i * n + j] = prevV[i * n + j];
          }
        }
      }
    }
  }

  void solveIncompressibility(int numIters, float dt, float overRelaxation,
                              bool compensateDrift = true) {
    std::fill(p.begin(), p.end(), 0.0f);
    prevU = u;
    prevV = v;

    int n = fNumY;
    float cp = density * h / dt;

    // for (int i = 0; i < fNumCells; i++) {
    //   float u = this->u[i];
    //   float v = this->v[i];
    // }

    for (int iter = 0; iter < numIters; iter++) {
      for (int i = 1; i < fNumX - 1; i++) {
        for (int j = 1; j < fNumY - 1; j++) {
          if (cellType[i * n + j] != CellType::FLUID)
            continue;

          int center = i * n + j;
          int left = (i - 1) * n + j;
          int right = (i + 1) * n + j;
          int bottom = i * n + j - 1;
          int top = i * n + j + 1;

          // float s = this->s[center];
          float sx0 = this->s[left];
          float sx1 = this->s[right];
          float sy0 = this->s[bottom];
          float sy1 = this->s[top];
          float s = sx0 + sx1 + sy0 + sy1;
          if (s == 0.0f)
            continue;

          float div = this->u[right] - this->u[center] + this->v[top] - this->v[center];

          if (particleRestDensity > 0.0f && compensateDrift) {
            float k = 1.0f;
            float compression = particleDensity[i * n + j] - particleRestDensity;
            if (compression > 0.0f)
              div = div - k * compression;
          }

          float p = -div / s;
          p *= overRelaxation;
          this->p[center] += cp * p;

          this->u[center] -= sx0 * p;
          this->u[right] += sx1 * p;
          this->v[center] -= sy0 * p;
          this->v[top] += sy1 * p;
        }
      }
    }
  }

  void updateParticleColors() {
    float h1 = fInvSpacing;

    for (int i = 0; i < numParticles; i++) {
      float s = 0.01;

      particleColor[3 * i] = std::clamp<float>(particleColor[3 * i] - s, 0.0, 1.0f);
      particleColor[3 * i + 1] = std::clamp<float>(particleColor[3 * i + 1] - s, 0.0, 1.0f);
      particleColor[3 * i + 1] = std::clamp<float>(particleColor[3 * i + 2] + s, 0.0, 1.0f);

      float x = particlePos[2 * i];
      float y = particlePos[2 * i + 1];
      int xi = std::clamp<int>(floor(x * h1), 1, fNumX - 1);
      int yi = std::clamp<int>(floor(y * h1), 1, fNumY - 1);
      int cellNr = xi * fNumY + yi;

      float d0 = particleRestDensity;

      if (d0 > 0.0) {
        float relDensity = particleDensity[cellNr] / d0;
        if (relDensity < 0.7) {
          float s = 0.8;
          particleColor[3 * i] = s;
          particleColor[3 * i + 1] = s;
          particleColor[3 * i + 2] = 1.0f;
        }
      }
    }
  }

  void setSciColor(int cellNr, float val, float minVal, float maxVal) {
    val = std::min(std::max(val, minVal), maxVal - 0.0001f);
    float d = maxVal - minVal;
    val = d == 0.0f ? 0.5f : (val - minVal) / d;
    float m = 0.25f;
    int num = std::floor(val / m);
    float s = (val - num * m) / m;
    float r = 0.0f, g = 0.0f, b = 0.0f;

    switch (num) {
    case 0:
      r = 0.0f;
      g = s;
      b = 1.0f;
      break;
    case 1:
      r = 0.0f;
      g = 1.0f;
      b = 1.0f - s;
      break;
    case 2:
      r = s;
      g = 1.0f;
      b = 0.0f;
      break;
    case 3:
      r = 1.0f;
      g = 1.0f - s;
      b = 0.0f;
      break;
    }

    cellColor[3 * cellNr] = r;
    cellColor[3 * cellNr + 1] = g;
    cellColor[3 * cellNr + 2] = b;
  }

  void updateCellColors() {
    std::fill(cellColor.begin(), cellColor.end(), 0.0f);

    for (int i = 0; i < fNumCells; i++) {
      if (cellType[i] == CellType::SOLID) {
        cellColor[3 * i] = 0.5f;
        cellColor[3 * i + 1] = 0.5f;
        cellColor[3 * i + 2] = 0.5f;
      } else if (cellType[i] == CellType::FLUID) {
        float d = particleDensity[i];
        if (particleRestDensity > 0.0f)
          d /= particleRestDensity;
        setSciColor(i, d, 0.0f, 2.0f);
      }
    }
  }

  void simulate(float dt, float gravity, float flipRatio, int numPressureIters,
                int numParticleIters, float overRelaxation, bool compensateDrift,
                bool separateParticles, float obstacleX, float obstacleY, float obstacleRadius) {
    int numSubSteps = 1;
    float sdt = dt / numSubSteps;

    for (int step = 0; step < numSubSteps; step++) {
      integrateParticles(sdt, gravity);
      if (separateParticles)
        pushParticlesApart(numParticleIters);
      handleParticleCollisions(obstacleX, obstacleY, obstacleRadius);
      transferVelocities(true, 0.0f); // TODO: what value here?
      updateParticleDensity();
      solveIncompressibility(numPressureIters, sdt, overRelaxation, compensateDrift);
      transferVelocities(false, flipRatio);
    }

    updateParticleColors();
    updateCellColors();
  }

  float density;
  int fNumX, fNumY;
  float h, fInvSpacing;
  int fNumCells;
  std::vector<float> u, v, du, dv, prevU, prevV, p, s;
  std::vector<int> cellType;
  std::vector<float> cellColor;
  int maxParticles;
  std::vector<float> particlePos, particleColor, particleVel;
  std::vector<float> particleDensity;
  float particleRestDensity, particleRadius;
  float pInvSpacing;
  int pNumX, pNumY, pNumCells;
  std::vector<int> numCellParticles, firstCellParticle, cellParticleIds;
  int numParticles;
}; // class FlipFluid
} // namespace fluid
