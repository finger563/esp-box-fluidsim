#include "render.hpp"

using namespace espp;

static espp::Logger logger({.tag = "render", .level = espp::Logger::Verbosity::WARN});
static std::unique_ptr<espp::Task> video_task;
static QueueHandle_t video_queue = nullptr;
static constexpr int num_rows_in_framebuffer = 50;

static bool video_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);

bool initialize_render() {
  if (video_queue != nullptr) {
    logger.error("video queue is already initialized!");
    return false;
  }

  video_queue = xQueueCreate(1, sizeof(uint16_t *));
  using namespace std::placeholders;
  video_task = espp::Task::make_unique({
      .callback = std::bind(video_task_callback, _1, _2, _3),
      .task_config =
          {.name = "video task", .stack_size_bytes = 4 * 1024, .priority = 20, .core_id = 1},
  });
  video_task->start();

  return true;
}

void IRAM_ATTR push_frame(const void *frame) {
  if (video_queue == nullptr) {
    logger.error("video queue is null, make sure to call initialize_video() first!");
    return;
  }
  xQueueSend(video_queue, &frame, 0);
}

bool video_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
  const void *_frame_ptr;
  if (xQueueReceive(video_queue, &_frame_ptr, portMAX_DELAY) != pdTRUE) {
    return false;
  }

  static const int num_lines_to_write = num_rows_in_framebuffer;
  static auto &box = EspBox::get();
  static uint16_t vram_index = 0; // has to be static so that it persists between calls
  static const int x_offset = 0;
  static const int y_offset = 0;
  static const int display_width = box.lcd_width();
  static const int display_height = box.lcd_height();
  static const int native_pitch = display_width;
  uint16_t *vram0 = (uint16_t *)box.vram0();
  uint16_t *vram1 = (uint16_t *)box.vram1();

  for (int y = 0; y < display_height; y += num_lines_to_write) {
    uint16_t *_buf =
        (uint16_t *)((uint32_t)vram0 * (vram_index ^ 0x01) + (uint32_t)vram1 * vram_index);
    vram_index = vram_index ^ 0x01;
    int num_lines = std::min<int>(num_lines_to_write, display_height - y);
    const uint16_t *_frame = (const uint16_t *)_frame_ptr;
    for (int i = 0; i < num_lines; i++) {
      // write two pixels (32 bits) at a time because it's faster
      for (int j = 0; j < display_width / 2; j++) {
        int src_index = (y + i) * native_pitch + j * 2;
        int dst_index = i * display_width + j * 2;
        // memcpy(&_buf[i*display_width + j * 2], &_frame[(y+i)*native_pitch + j * 2], 4);
        _buf[dst_index] = _frame[src_index];
        _buf[dst_index + 1] = _frame[src_index + 1];
      }
    }
    box.write_lcd_frame(x_offset, y + y_offset, display_width, num_lines, (uint8_t *)&_buf[0]);
  }
  return false;
}
