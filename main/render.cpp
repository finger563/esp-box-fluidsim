#include "render.hpp"

using namespace espp;

static espp::Logger logger({.tag = "render", .level = espp::Logger::Verbosity::WARN});
static std::unique_ptr<espp::Task> video_task;
static QueueHandle_t video_queue = nullptr;
static constexpr int num_rows_in_framebuffer = 50;

static bool video_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);
static void draw_circle(int cx, int cy, int radius, uint16_t color, uint16_t *framebuffer);

static const int display_width = EspBox::lcd_width();
static const int display_height = EspBox::lcd_height();

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
          {.name = "video task", .stack_size_bytes = 4 * 1024, .priority = 20, .core_id = 0},
  });
  video_task->start();

  return true;
}

bool IRAM_ATTR push_frame(const void *frame) {
  if (video_queue == nullptr) {
    logger.error("video queue is null, make sure to call initialize_video() first!");
    return false;
  }
  return xQueueSend(video_queue, &frame, 0) == pdTRUE;
}

void render_rectangle(int cx, int cy, int width, int height, uint16_t color,
                      uint16_t *framebuffer) {
  int x0 = std::clamp<int>(cx - width / 2, 0, display_width - 1);
  int x1 = std::clamp<int>(cx + width / 2, 0, display_width - 1);
  int y0 = std::clamp<int>(cy - height / 2, 0, display_height - 1);
  int y1 = std::clamp<int>(cy + height / 2, 0, display_height - 1);
  for (int fby = y0; fby <= y1; fby++) {
    for (int fbx = x0; fbx <= x1; fbx++) {
      framebuffer[fby * display_width + fbx] = color;
    }
  }
}

void render_rectancle(int x, int y, int width, int height, uint16_t color, bool filled, bool border,
                      uint16_t border_color, uint16_t *framebuffer) {
  if (width <= 0 || height <= 0) {
    return;
  }
  int xs = std::clamp(x, 0, display_width - 1);
  int ys = std::clamp(y, 0, display_height - 1);
  int xe = std::clamp(x + width, 0, display_width - 1);
  int ye = std::clamp(y + height, 0, display_height - 1);
  if (filled) {
    for (int x = xs; x < xe; x++) {
      for (int y = ys; y < ye; y++) {
        framebuffer[y * display_width + x] = color;
      }
    }
  } else {
    for (int y = ys; y < ye; y++) {
      for (int x = xs; x < xe; x++) {
        if (border && (y == ys || y == ye || x == xs || x == xe)) {
          framebuffer[y * display_width + x] = border_color;
        } else {
          framebuffer[y * display_width + x] = color;
        }
      }
    }
  }
}

void render_circle(int x, int y, int radius, uint16_t color, uint16_t *framebuffer) {
  if (radius <= 0) {
    return;
  }
  draw_circle(x, y, radius, color, framebuffer);
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

static void draw_circle(int cx, int cy, int radius, uint16_t color, uint16_t *framebuffer) {
  int xs = std::clamp(cx - radius, 0, display_width - 1);
  int xe = std::clamp(cx + radius, 0, display_width - 1);
  int radius_sqr = radius * radius;
  for (int x = xs; x < xe; x++) {
    int loc_x = x - cx;
    int height = (int)std::sqrt(radius_sqr - loc_x * loc_x);
    int ys = std::clamp(cy - height, 0, display_height - 1);
    int ye = std::clamp(cy + height, 0, display_height - 1);
    for (int y = ys; y < ye; y++)
      framebuffer[y * display_width + x] = color;
  }
}
