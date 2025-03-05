#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "color.hpp"
#include "fast_math.hpp"
#include "vector2d.hpp"

/// This file contains a simple FLIP fluid simulation implementation. It is
/// based on the presentation / video by Matthias Müller-Fischer:
/// - https://youtu.be/XmzBREkK8kY
/// - https://github.com/matthias-research/pages/blob/master/tenMinutePhysics/18-flip.html
///
/// The implementation is not optimized for performance, but for simplicity and
/// readability. It is not meant to be used in production code, but as a
/// reference for understanding the basics of FLIP simulations.
namespace fluid {
enum class CellType {
  FLUID,
  AIR,
  SOLID,
}; // enum class CellType

struct Particle {
  espp::Vector2f pos;
  espp::Vector2f vel;
  float density;
  int cell_index;
  espp::Rgb color;
}; // struct Particle

struct GridCell {
  float u, v;
  float du, dv;
  float prev_u, prev_v;
  float p;
  float s;
  int first_particle_id;
  int num_particles;
  CellType type;
  espp::Rgb color;
}; // struct GridCell

struct Fluid {
  float density{0.0f};
  std::vector<Particle> particles{};
  std::vector<GridCell> grid{};
  std::vector<size_t> cell_particle_ids{};
  int width{0};
  int height{0};

  int num_x{0};
  int num_y{0};
  float h{0.0f};
  float inv_spacing{0.0f};
  size_t num_cells{0};
  int max_particles{0};

  int num_particles{0};

  explicit Fluid(float density, int width, int height, float spacing, float particle_radius,
                 int max_particles)
      : density(density)
      , width(width)
      , height(height)
      , max_particles(max_particles) {
    num_x = std::floor(width / spacing) + 1;
    num_y = std::floor(height / spacing) + 1;
    h = std::max(width / num_x, height / num_y);
    inv_spacing = 1.0f / h;
    num_cells = num_x * num_y;
    grid.resize(num_cells);
    particles.reserve(max_particles);
    cell_particle_ids.resize(max_particles);
  }
}; // struct Fluid

class FlipFluid {
public:
  explicit FlipFluid(float density, size_t width, size_t height, float spacing,
                     float particle_radius, size_t max_particles)
      : fluid_(density, width, height, spacing, particle_radius, max_particles)
      , rest_density(density)
      , particle_radius(particle_radius)
      , inv_spacing(1.0f / spacing) {}

  void integrate_particles(float dt, const espp::Vector2f &gravity) {
    for (auto &p : fluid_.particles) {
      p.vel += gravity * dt;
      p.pos += p.vel * dt;
    }
  }

  void push_particles_apart(size_t num_iterations) {
    static constexpr float color_diffusion_coeff = 0.001f;
    const float min_dist = 2.0f * particle_radius;
    const float min_dist2 = min_dist * min_dist;
    // Count particles per cell
    for (int i = 0; i < fluid_.num_particles; i++) {
      const auto &p = fluid_.particles[i];
      const auto xi =
          std::clamp(static_cast<int>(p.pos.x() * fluid_.inv_spacing), 0, fluid_.num_x - 1);
      const auto yi =
          std::clamp(static_cast<int>(p.pos.y() * fluid_.inv_spacing), 0, fluid_.num_y - 1);
      const auto cell_nr = xi * fluid_.num_y + yi;
      fluid_.grid[cell_nr].num_particles++;
    }

    // Partial sums
    int first = 0;
    for (int i = 0; i < fluid_.num_cells; i++) {
      first += fluid_.grid[i].num_particles;
      fluid_.grid[i].first_particle_id = first;
    }
    // fluid_.grid[fluid_.num_cells].first_particle_id = first; // guard

    // Fill particles into cells
    for (int i = 0; i < fluid_.num_particles; i++) {
      const auto &p = fluid_.particles[i];
      const auto xi =
          std::clamp(static_cast<int>(p.pos.x() * fluid_.inv_spacing), 0, fluid_.num_x - 1);
      const auto yi =
          std::clamp(static_cast<int>(p.pos.y() * fluid_.inv_spacing), 0, fluid_.num_y - 1);
      const auto cell_nr = xi * fluid_.num_y + yi;
      fluid_.grid[cell_nr].first_particle_id--;
      fluid_.cell_particle_ids[fluid_.grid[cell_nr].first_particle_id] = i;
    }

    // Push particles apart
    for (int iter = 0; iter < num_iterations; iter++) {
      for (int i = 0; i < fluid_.num_particles; i++) {
        const auto &p = fluid_.particles[i];
        const auto xi =
            std::clamp(static_cast<int>(p.pos.x() * fluid_.inv_spacing), 0, fluid_.num_x - 1);
        const auto yi =
            std::clamp(static_cast<int>(p.pos.y() * fluid_.inv_spacing), 0, fluid_.num_y - 1);
        const auto x0 = std::max(xi - 1, 0);
        const auto y0 = std::max(yi - 1, 0);
        const auto x1 = std::min(xi + 1, fluid_.num_x - 1);
        const auto y1 = std::min(yi + 1, fluid_.num_y - 1);
        for (int xi = x0; xi <= x1; xi++) {
          for (int yi = y0; yi <= y1; yi++) {
            const auto cell_nr = xi * fluid_.num_y + yi;
            const auto first = fluid_.grid[cell_nr].first_particle_id;
            const auto last = first + fluid_.grid[cell_nr].num_particles;
            for (int j = first; j < last; j++) {
              const auto id = fluid_.cell_particle_ids[j];
              if (id == i) {
                continue;
              }
              const auto &q = fluid_.particles[id];
              const auto dx = q.pos.x() - p.pos.x();
              const auto dy = q.pos.y() - p.pos.y();
              const auto d2 = dx * dx + dy * dy;
              if (d2 > min_dist2 || d2 == 0.0f) {
                continue;
              }
              const auto d = std::sqrt(d2);
              const auto s = 0.5f * (min_dist - d) / d;
              const auto dxs = dx * s;
              const auto dys = dy * s;
              fluid_.particles[i].pos.x(fluid_.particles[i].pos.x() - dxs);
              fluid_.particles[i].pos.y(fluid_.particles[i].pos.y() - dys);
              fluid_.particles[id].pos.x(fluid_.particles[id].pos.x() + dxs);
              fluid_.particles[id].pos.y(fluid_.particles[id].pos.y() + dys);
              // TODO: Diffuse colors
              // const auto color0 = fluid_.particles[i].color;
              // const auto color1 = fluid_.particles[id].color;
              // const auto color = color0 + color1;
              // fluid_.particles[i].color = color0 + (color - color0) * color_diffusion_coeff;
              // fluid_.particles[id].color = color1 + (color - color1) * color_diffusion_coeff;
            }
          }
        }
      }
    }
  }

  void handle_particle_collisions(const espp::Vector2f &obstacle_pos, float obstacle_radius) {
    const auto h = 1.0f / fluid_.inv_spacing;
    const auto r = particle_radius;
    const auto or2 = obstacle_radius * obstacle_radius;
    const auto min_dist = obstacle_radius + r;
    const auto min_dist2 = min_dist * min_dist;
    const auto min_x = h + r;
    const auto max_x = (fluid_.num_x - 1) * h - r;
    const auto min_y = h + r;
    const auto max_y = (fluid_.num_y - 1) * h - r;
    for (int i = 0; i < fluid_.num_particles; i++) {
      auto &p = fluid_.particles[i];
      auto dx = p.pos.x() - obstacle_pos.x();
      auto dy = p.pos.y() - obstacle_pos.y();
      auto d2 = dx * dx + dy * dy;
      // Obstacle collision
      if (d2 < min_dist2) {
        p.vel = espp::Vector2f(0.0f, 0.0f);
      }
      // Wall collisions
      if (p.pos.x() < min_x) {
        p.pos.x(min_x);
        p.vel.x(0.0f);
      }
      if (p.pos.x() > max_x) {
        p.pos.x(max_x);
        p.vel.x(0.0f);
      }
      if (p.pos.y() < min_y) {
        p.pos.y(min_y);
        p.vel.y(0.0f);
      }
      if (p.pos.y() > max_y) {
        p.pos.y(max_y);
        p.vel.y(0.0f);
      }
    }
  }

  void update_particle_density() {
    const auto n = fluid_.num_y;
    const auto h = fluid_.h;
    const auto h1 = fluid_.inv_spacing;
    const auto h2 = 0.5f * h;
    for (int i = 0; i < fluid_.num_particles; i++) {
      auto &p = fluid_.particles[i];
      auto x = p.pos.x();
      auto y = p.pos.y();
      x = std::clamp(x, h, (fluid_.num_x - 1) * h);
      y = std::clamp(y, h, (fluid_.num_y - 1) * h);
      const auto x0 = std::floor((x - h2) * h1);
      const auto tx = ((x - h2) - x0 * h) * h1;
      const auto x1 = std::min<int>(x0 + 1, fluid_.num_x - 2);
      const auto y0 = std::floor((y - h2) * h1);
      const auto ty = ((y - h2) - y0 * h) * h1;
      const auto y1 = std::min<int>(y0 + 1, fluid_.num_y - 2);
      const auto sx = 1.0f - tx;
      const auto sy = 1.0f - ty;
      if (x0 < fluid_.num_x && y0 < fluid_.num_y) {
        fluid_.grid[x0 * n + y0].s += sx * sy;
      }
      if (x1 < fluid_.num_x && y0 < fluid_.num_y) {
        fluid_.grid[x1 * n + y0].s += tx * sy;
      }
      if (x1 < fluid_.num_x && y1 < fluid_.num_y) {
        fluid_.grid[x1 * n + y1].s += tx * ty;
      }
      if (x0 < fluid_.num_x && y1 < fluid_.num_y) {
        fluid_.grid[x0 * n + y1].s += sx * ty;
      }
    }
    if (rest_density == 0.0f) {
      float sum = 0.0f;
      int num_fluid_cells = 0;
      for (int i = 0; i < fluid_.num_cells; i++) {
        if (fluid_.grid[i].type == CellType::FLUID) {
          sum += fluid_.grid[i].s;
          num_fluid_cells++;
        }
      }
      if (num_fluid_cells > 0) {
        rest_density = sum / num_fluid_cells;
      }
    }
  }

  void transfer_velocities(bool to_grid, float flip_ratio = 0.0f) {
    const auto n = fluid_.num_y;
    const auto h = fluid_.h;
    const auto h1 = fluid_.inv_spacing;
    const auto h2 = 0.5f * h;
    if (to_grid) {
      for (auto &cell : fluid_.grid) {
        cell.prev_u = cell.u;
        cell.prev_v = cell.v;
        cell.du = 0.0f;
        cell.dv = 0.0f;
        cell.u = 0.0f;
        cell.v = 0.0f;
        cell.type = cell.s == 0.0f ? CellType::SOLID : CellType::AIR;
      }
      for (int i = 0; i < fluid_.num_particles; i++) {
        const auto &p = fluid_.particles[i];
        const auto xi =
            std::clamp(static_cast<int>(p.pos.x() * fluid_.inv_spacing), 0, fluid_.num_x - 1);
        const auto yi =
            std::clamp(static_cast<int>(p.pos.y() * fluid_.inv_spacing), 0, fluid_.num_y - 1);
        const auto cell_nr = xi * n + yi;
        if (fluid_.grid[cell_nr].type == CellType::AIR) {
          fluid_.grid[cell_nr].type = CellType::FLUID;
        }
      }
    }
    for (int component = 0; component < 2; component++) {
      const auto dx = component == 0 ? 0.0f : h2;
      const auto dy = component == 0 ? h2 : 0.0f;
      auto &f = component == 0 ? fluid_.grid[0].u : fluid_.grid[0].v;
      auto &prev_f = component == 0 ? fluid_.grid[0].prev_u : fluid_.grid[0].prev_v;
      auto &d = component == 0 ? fluid_.grid[0].du : fluid_.grid[0].dv;
      for (int i = 0; i < fluid_.num_particles; i++) {
        const auto &p = fluid_.particles[i];
        const auto x = p.pos.x();
        const auto y = p.pos.y();
        const auto xi = std::clamp(static_cast<int>(x * fluid_.inv_spacing), 0, fluid_.num_x - 1);
        const auto yi = std::clamp(static_cast<int>(y * fluid_.inv_spacing), 0, fluid_.num_y - 1);

        const auto x0 = std::min(std::max(xi - 1, 0), fluid_.num_x - 1);
        const auto tx = (x - (x0 * h)) * h1;
        const auto x1 = std::min(xi + 1, fluid_.num_x - 1);

        const auto y0 = std::min(std::max(yi - 1, 0), fluid_.num_y - 1);
        const auto ty = (y - (y0 * h)) * h1;
        const auto y1 = std::min(yi + 1, fluid_.num_y - 1);

        const auto sx = 1.0f - tx;
        const auto sy = 1.0f - ty;
        const auto d0 = sx * sy;
        const auto d1 = tx * sy;
        const auto d2 = tx * ty;
        const auto d3 = sx * ty;
        const auto nr0 = x0 * n + y0;
        const auto nr1 = x1 * n + y0;
        const auto nr2 = x1 * n + y1;
        const auto nr3 = x0 * n + y1;

        if (to_grid) {
          const auto pv = component == 0 ? p.vel.x() : p.vel.y();
          f[nr0] += pv * d0;
          d[nr0] += d0;
          f[nr1] += pv * d1;
          d[nr1] += d1;
          f[nr2] += pv * d2;
          d[nr2] += d2;
          f[nr3] += pv * d3;
          d[nr3] += d3;
        } else {
          const auto offset = component == 0 ? n : 1;
          const auto valid0 = fluid_.grid[nr0].type != CellType::AIR ||
                                      fluid_.grid[nr0 - offset].type != CellType::AIR
                                  ? 1.0f
                                  : 0.0f;
          const auto valid1 = fluid_.grid[nr1].type != CellType::AIR ||
                                      fluid_.grid[nr1 - offset].type != CellType::AIR
                                  ? 1.0f
                                  : 0.0f;
          const auto valid2 = fluid_.grid[nr2].type != CellType::AIR ||
                                      fluid_.grid[nr2 - offset].type != CellType::AIR
                                  ? 1.0f
                                  : 0.0f;
          const auto valid3 = fluid_.grid[nr3].type != CellType::AIR ||
                                      fluid_.grid[nr3 - offset].type != CellType::AIR
                                  ? 1.0f
                                  : 0.0f;
          const auto v = component == 0 ? p.vel.x() : p.vel.y();
          const auto d = valid0 * d0 + valid1 * d1 + valid2 * d2 + valid3 * d3;
          if (d > 0.0f) {
            const auto pic_v = (valid0 * d0 * f[nr0] + valid1 * d1 * f[nr1] + valid2 * d2 * f[nr2] +
                                valid3 * d3 * f[nr3]) /
                               d;
            const auto corr =
                (valid0 * d0 * (f[nr0] - prev_f[nr0]) + valid1 * d1 * (f[nr1] - prev_f[nr1]) +
                 valid2 * d2 * (f[nr2] - prev_f[nr2]) + valid3 * d3 * (f[nr3] - prev_f[nr3])) /
                d;
            const auto flip_v = v + corr;
            fluid_.particles[i].vel.x((1.0f - flip_ratio) * pic_v + flip_ratio * flip_v);
          }
        }
      }
      if (to_grid) {
        for (int i = 0; i < fluid_.num_cells; i++) {
          if (d[i] > 0.0f) {
            f[i] /= d[i];
          }
        }
        for (int i = 0; i < fluid_.num_x; i++) {
          for (int j = 0; j < fluid_.num_y; j++) {
            const auto solid = fluid_.grid[i * n + j].type == CellType::SOLID;
            if (solid || (i > 0 && fluid_.grid[(i - 1) * n + j].type == CellType::SOLID)) {
              fluid_.grid[i * n + j].u = fluid_.grid[i * n + j].prev_u;
            }
            if (solid || (j > 0 && fluid_.grid[i * n + j - 1].type == CellType::SOLID)) {
              fluid_.grid[i * n + j].v = fluid_.grid[i * n + j].prev_v;
            }
          }
        }
      }
    }
  }

  void solve_incompressibility(size_t num_iterations, float dt, float over_relaxation,
                               bool compensate_drift = true) {
    const auto cp = fluid_.density * fluid_.h / dt;
    for (int iter = 0; iter < num_iterations; iter++) {
      for (int i = 1; i < fluid_.num_x - 1; i++) {
        for (int j = 1; j < fluid_.num_y - 1; j++) {
          if (fluid_.grid[i * fluid_.num_y + j].type != CellType::FLUID) {
            continue;
          }
          const auto center = i * fluid_.num_y + j;
          const auto left = (i - 1) * fluid_.num_y + j;
          const auto right = (i + 1) * fluid_.num_y + j;
          const auto bottom = i * fluid_.num_y + j - 1;
          const auto top = i * fluid_.num_y + j + 1;
          const auto s = fluid_.grid[center].s;
          const auto sx0 = fluid_.grid[left].s;
          const auto sx1 = fluid_.grid[right].s;
          const auto sy0 = fluid_.grid[bottom].s;
          const auto sy1 = fluid_.grid[top].s;
          const auto div = fluid_.grid[right].u - fluid_.grid[center].u + fluid_.grid[top].v -
                           fluid_.grid[center].v;
          if (fluid_.grid[center].type == CellType::FLUID) {
            const auto p = -div / s;
            fluid_.grid[center].p += cp * p;
            fluid_.grid[center].u -= sx0 * p;
            fluid_.grid[right].u += sx1 * p;
            fluid_.grid[center].v -= sy0 * p;
            fluid_.grid[top].v += sy1 * p;
          }
        }
      }
    }
  }

  void update_particle_colors() {}

  void set_sci_color(size_t cell_index, float value, float min_value, float max_value) {}

  void update_cell_colors() {}

  void simulate(float dt, const espp::Vector2f &gravity, float flip_ratio,
                size_t num_pressure_iterations, size_t num_particle_iterations,
                float over_relaxation, bool compensate_drift, bool separate_particles,
                const espp::Vector2f &obstacle_pos, float obstacle_radius) {
    const auto num_sub_steps = 1;
    const auto sdt = dt / num_sub_steps;
    for (int step = 0; step < num_sub_steps; step++) {
      integrate_particles(sdt, gravity);
      if (separate_particles) {
        push_particles_apart(num_particle_iterations);
      }
      handle_particle_collisions(obstacle_pos, obstacle_radius);
      transfer_velocities(true);
      update_particle_density();
      solve_incompressibility(num_pressure_iterations, sdt, over_relaxation, compensate_drift);
      transfer_velocities(false, flip_ratio);
    }

    update_particle_colors();
    update_cell_colors();
  }

protected:
  Fluid fluid_;

  float rest_density;
  float particle_radius;
  float inv_spacing;
}; // class FlipFluid

} // namespace fluid
