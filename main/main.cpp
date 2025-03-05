#include <chrono>
#include <thread>

#include "esp-box.hpp"
#include "kalman_filter.hpp"
#include "logger.hpp"
#include "madgwick_filter.hpp"
#include "timer.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Fluidsim", .level = espp::Logger::Verbosity::DEBUG});

  logger.info("Bootup");

  espp::EspBox &box = espp::EspBox::get();
  box.set_log_level(espp::Logger::Verbosity::INFO);
  logger.info("Running on {}", box.box_type());

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
        // TODO: interact with the fluid sim
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

  // make a task to read out the IMU data and print it to console
  using namespace std::chrono_literals;
  espp::Timer imu_timer(
      {.period = 10ms,
          .callback = []() -> bool {
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
            auto temp = imu->get_temperature(ec);

            // with only the accelerometer + gyroscope, we can't get yaw :(
            float roll = 0, pitch = 0, yaw = 0; // NOTE:yaw is unused
            static constexpr float beta = 0.1f; // higher = more accelerometer, lower = more gyro
            static espp::MadgwickFilter f(beta);

            // update the state
            f.update(dt, accel.x, accel.y, accel.z, gyro.x * M_PI / 180.0f, gyro.y * M_PI / 180.0f,
                     gyro.z * M_PI / 180.0f);
            f.get_euler(roll, pitch, yaw);
            pitch *= M_PI / 180.0f;
            roll *= M_PI / 180.0f;

            // compute the gravity vector
            float vx = sin(pitch);
            float vy = -cos(pitch) * sin(roll);
            float vz = -cos(pitch) * cos(roll);

            // TODO: provide the updated gravity vector to the fluid sim

            return false;
          },
          .task_config = {
            .name = "IMU",
            .stack_size_bytes = 6 * 1024,
            .priority = 10,
            .core_id = 0,
          }});
  imu_timer.start();

  // loop forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
