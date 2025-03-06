#include <chrono>
#include <thread>

#include "esp-box.hpp"
#include "kalman_filter.hpp"
#include "logger.hpp"
#include "madgwick_filter.hpp"
#include "timer.hpp"
#include "vector2d.hpp"

#include "fluidsim.hpp"
#include "render.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Fluidsim", .level = espp::Logger::Verbosity::INFO});

  logger.info("Bootup");

  espp::EspBox &box = espp::EspBox::get();
  box.set_log_level(espp::Logger::Verbosity::INFO);
  logger.info("Running on {}", box.box_type());

  // mutex for the touch position
  std::mutex touch_mutex;
  espp::Vector2f touch_pos{box.lcd_width() / 2, box.lcd_height() / 2};
  bool new_touch = false;

  auto touch_callback = [&](const auto &touch) {
    // NOTE: since we're directly using the touchpad data, and not using the
    // TouchpadInput + LVGL, we'll need to ensure the touchpad data is
    // converted into proper screen coordinates instead of simply using the
    // raw values.
    static auto previous_touchpad_data = box.touchpad_convert(touch);
    auto touchpad_data = box.touchpad_convert(touch);
    if (touchpad_data != previous_touchpad_data) {
      logger.debug("Touch: {}", touchpad_data);
      previous_touchpad_data = touchpad_data;
      // if the button is pressed, reset the fluid sim
      if (touchpad_data.btn_state) {
        // TODO: reset the fluid sim
      }
      // if there is a touch point, use it to interact with the fluid sim
      if (touchpad_data.num_touch_points > 0) {
        std::lock_guard<std::mutex> lock(touch_mutex);
        touch_pos = espp::Vector2f(touchpad_data.x, touchpad_data.y);
        new_touch = true;
      }
    }
  };

  // initialize the sound
  if (!box.initialize_sound()) {
    logger.error("Failed to initialize sound!");
    return;
  }
  // initialize the LCD
  if (!box.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be 50 lines high
  static constexpr size_t pixel_buffer_size = box.lcd_width() * 50;
  espp::Task::BaseConfig display_task_config = {
      .name = "Display",
      .stack_size_bytes = 6 * 1024,
      .priority = 10,
      .core_id = 0,
  };
  // initialize the LVGL display for the esp-box
  if (!box.initialize_display(pixel_buffer_size, display_task_config)) {
    logger.error("Failed to initialize display!");
    return;
  }
  // NOTE: we pause the display here so that the lvgl display task does not run.
  // We will manually draw to the screen and bypass lvgl for this demo.
  auto display = box.display();
  display->pause();

  // start the render task
  if (!initialize_render()) {
    logger.error("Failed to initialize render!");
    return;
  }

  // initialize the touchpad
  if (!box.initialize_touch(touch_callback)) {
    logger.error("Failed to initialize touchpad!");
    return;
  }

  // initialize the IMU
  if (!box.initialize_imu()) {
    logger.error("Failed to initialize IMU!");
    return;
  }

  // unmute the audio and set the volume to 60%
  box.mute(false);
  box.volume(60.0f);

  // set the display brightness to be 75%
  box.brightness(75.0f);

  // gravity vector to be used with the fluid sim
  std::mutex gravity_mutex;
  std::array<float, 3> gravity = {0, 0, 0};

  // make a task to read out the IMU data and print it to console
  using namespace std::chrono_literals;
  espp::Timer imu_timer({.period = 10ms,
                         .callback = [&gravity, &gravity_mutex]() -> bool {
                           static auto &box = espp::EspBox::get();
                           static auto imu = box.imu();

                           auto now = esp_timer_get_time(); // time in microseconds
                           static auto t0 = now;
                           auto t1 = now;
                           float dt = (t1 - t0) / 1'000'000.0f; // convert us to s
                           t0 = t1;

                           std::error_code ec;
                           // get imu data
                           auto accel = imu->get_accelerometer(ec);
                           auto gyro = imu->get_gyroscope(ec);
                           // auto temp = imu->get_temperature(ec);

                           // with only the accelerometer + gyroscope, we can't get yaw :(
                           float roll = 0, pitch = 0, yaw = 0; // NOTE:yaw is unused
                           static constexpr float beta =
                               0.1f; // higher = more accelerometer, lower = more gyro
                           static espp::MadgwickFilter f(beta);

                           // update the state
                           f.update(dt, accel.x, accel.y, accel.z, gyro.x * M_PI / 180.0f,
                                    gyro.y * M_PI / 180.0f, gyro.z * M_PI / 180.0f);
                           f.get_euler(roll, pitch, yaw);
                           pitch *= M_PI / 180.0f;
                           roll *= M_PI / 180.0f;

                           // compute the gravity vector and store it
                           float gx = sin(pitch);
                           float gy = -cos(pitch) * sin(roll);
                           float gz = -cos(pitch) * cos(roll);
                           std::lock_guard<std::mutex> lock(gravity_mutex);
                           gravity[0] = gx;
                           gravity[1] = gy;
                           gravity[2] = gz;

                           return false;
                         },
                         .auto_start = true,
                         .task_config = {
                             .name = "IMU",
                             .stack_size_bytes = 6 * 1024,
                             .priority = 10,
                             .core_id = 0,
                         }});

  // now make the fluid sim
  static constexpr float density = 1000.0f;
  static constexpr int sim_width = box.lcd_width() / 2;
  static constexpr int sim_height = box.lcd_height() / 2;
  static constexpr int resolution = 40;
  static constexpr float spacing = sim_height / resolution;
  static constexpr float particle_radius = spacing * 0.3;
  float dx = 2.0 * particle_radius;
  float dy = sqrt(3.0) / 2.0 * dx;
  float relWaterWidth = 0.2;
  float relWaterHeight = 0.8;
  int numX = floor((relWaterWidth * sim_width - 2.0 * spacing - 2.0 * particle_radius) / dx);
  int numY = floor((relWaterHeight * sim_height - 2.0 * spacing - 2.0 * particle_radius) / dy);
  int max_particles = numX * numY;

  // static constexpr int numX = width / spacing;
  // static constexpr int max_particles = 100;

  logger.info("Creating fluid sim\n"
              "\t{} particles\n"
              "\t{} width, {} height\n"
              "\t{} x, {} y\n"
              "\t{} particle radius\n"
              "\t{} spacing",
              max_particles, sim_width, sim_height, numX, numY, particle_radius, spacing);

  std::shared_ptr<fluid::FlipFluid> fluid = std::make_shared<fluid::FlipFluid>(
      density, sim_width, sim_height, spacing, particle_radius, max_particles);

  static constexpr float g = -9.81;
  static constexpr float flip_ratio = 0.9;
  static constexpr int num_pressure_iters = 50;
  static constexpr int num_particle_iters = 2;
  static constexpr float over_relaxation = 1.9;
  static constexpr bool compensate_drift = true;
  static constexpr bool separate_particles = true;
  static constexpr float obstacle_radius = 10.0f;

  static constexpr int lcd_width = box.lcd_width();
  static constexpr int lcd_height = box.lcd_height();
  static constexpr int framebuffer_size = lcd_width * lcd_height * sizeof(uint16_t);

  // create particles
  fluid->numParticles = numX * numY;
  int p = 0;
  for (int i = 0; i < numX; i++) {
    for (int j = 0; j < numY; j++) {
      fluid->particlePos[p++] =
          spacing + particle_radius + dx * i + (j % 2 == 0 ? 0.0 : particle_radius);
      fluid->particlePos[p++] = spacing + particle_radius + dy * j;
    }
  }

  // setup grid cells for tank
  int n = fluid->fNumY;
  for (int i = 0; i < fluid->fNumX; i++) {
    for (int j = 0; j < fluid->fNumY; j++) {
      int s = 1.0; // fluid
      if (i == 0 || i == fluid->fNumX - 1 || j == 0 || j == fluid->fNumY - 1)
        s = 0.0; // solid
      fluid->s[i * n + j] = s;
    }
  }

  espp::Vector2f obstacle_pos{};
  {
    std::lock_guard<std::mutex> lock(touch_mutex);
    // get obstacle position
    obstacle_pos = touch_pos;
  }
  // invert the y position
  obstacle_pos.y(lcd_height - obstacle_pos.y());
  // convert to sim coordinates
  obstacle_pos /= spacing;
  // tell the sim where the obstacle is
  fluid->set_obstacle(obstacle_pos.x(), obstacle_pos.y(), obstacle_radius, true, 0);

  // now make a task to update the fluid sim
  espp::Timer fluid_timer(
      {.period = 5ms,
       .callback = [&new_touch, &touch_pos, &touch_mutex, &gravity, &gravity_mutex,
                    &fluid]() -> bool {
         // get the gravity vector
         std::array<float, 3> gravity_vector{};
         {
           std::lock_guard<std::mutex> lock(gravity_mutex);
           gravity_vector = gravity;
         }

         static auto prev_time = esp_timer_get_time();
         auto time = esp_timer_get_time();
         float dt = (time - prev_time) / 1'000'000.0f; // convert us to s
         prev_time = time;
         // fmt::print("dt: {:.3f}\n", dt);

         // get the obstacle position (where the user is touching)
         espp::Vector2f obstacle_pos{};
         {
           std::lock_guard<std::mutex> lock(touch_mutex);
           // get obstacle position
           obstacle_pos = touch_pos;
           new_touch = false;
         }
         // invert the y position
         obstacle_pos.y(lcd_height - obstacle_pos.y());
         // convert to sim coordinates
         obstacle_pos /= spacing;
         // tell the sim where the obstacle is
         fluid->set_obstacle(obstacle_pos.x(), obstacle_pos.y(), obstacle_radius, false, dt);

         // update the fluid sim
         fluid->simulate(4 * dt, // NOTE: run faster than real time not because the sim is
                                 // slow, but because it looks more realistic
                         -g * gravity_vector[0], // NOTE: we invert the x to match the
                                                 // simulation coords
                         g * gravity_vector[1], flip_ratio, num_pressure_iters, num_particle_iters,
                         over_relaxation, compensate_drift, separate_particles, obstacle_pos.x(),
                         obstacle_pos.y(), obstacle_radius);

         // ping pong between the two full frame buffers
         static int current_buffer = 0;
         static auto &box = espp::EspBox::get();
         static uint16_t *framebuffers[2] = {(uint16_t *)box.frame_buffer0(),
                                             (uint16_t *)box.frame_buffer1()};
         uint16_t *framebuffer = framebuffers[current_buffer];
         current_buffer = current_buffer ? 0 : 1;
         if (!framebuffer) {
           fmt::print("No framebuffer, make sure you have SPIRAM enabled!\n");
           return false;
         }
         // clear the frame buffer
         memset(framebuffer, 0, framebuffer_size);

         // draw the obstacle as a circle
         {
           float x = obstacle_pos.x() * spacing;
           float y = lcd_height - obstacle_pos.y() * spacing;
           uint16_t color = lv_color_to_u16(lv_color_make(255, 255, 255));
           render_circle(x, y, obstacle_radius * spacing, color, framebuffer);
         }

         // render to the active frame buffer
         float particle_render_radius = particle_radius * spacing;
         for (int i = 0; i < fluid->numParticles; i++) {
           float x = fluid->particlePos[2 * i] * spacing;
           float y = lcd_height - fluid->particlePos[2 * i + 1] * spacing;

           uint8_t red = fluid->particleColor[3 * i] * 255;
           uint8_t green = fluid->particleColor[3 * i + 1] * 255;
           uint8_t blue = fluid->particleColor[3 * i + 2] * 255;
           uint16_t color = lv_color_to_u16(lv_color_make(red, green, blue));

           render_circle(x, y, particle_render_radius, color, framebuffer);
           // render_rectangle(x, y, particle_render_radius, particle_render_radius, color,
           // framebuffer);
         }

         // then push the frame to the render task using push_frame
         if (!push_frame(framebuffer)) {
           // fmt::print("Failed to push frame to render task!\n");
         }

         return false;
       },
       .auto_start = true,
       .task_config =
           {
               .name = "Fluid",
               .stack_size_bytes = 6 * 1024,
               .priority = 10,
               .core_id = 1,
           },
       .log_level = espp::Logger::Verbosity::ERROR});

  // loop forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
