#pragma once

#include "esp-box.hpp"

bool initialize_render();
void render_rectangle(int cx, int cy, int width, int height, uint16_t color, uint16_t *framebuffer);
void render_rectancle(int x, int y, int width, int height, uint16_t color, bool filled, bool border,
                      uint16_t border_color, uint16_t *framebuffer);
void render_circle(int x, int y, int radius, uint16_t color, uint16_t *framebuffer);
bool push_frame(const void *frame);
