// Renders screen 2's kick view on the desktop, through the same
// drawKickShape() the firmware uses, to PBM images for checking the layout.
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include "KickShapeModel.h"
#include FONT_SOURCE  // Adafruit GFX glcdfont.c: `static const unsigned char font[]`

struct FakeOled {
  uint8_t px[64][128] = {};
  int cx = 0, cy = 0, size = 1; uint16_t color = 1;
  void clearDisplay(){ memset(px, 0, sizeof(px)); }
  void setTextSize(int s){ size = s; }
  void setTextColor(uint16_t c){ color = c; }
  void setCursor(int x, int y){ cx = x; cy = y; }
  void drawPixel(int x, int y, uint16_t c){
    if (x < 0 || x > 127 || y < 0 || y > 63) return;
    if (c == 2) px[y][x] ^= 1; else px[y][x] = c ? 1 : 0;
  }
  void drawFastHLine(int x, int y, int w, uint16_t c){ for (int i = 0; i < w; i++) drawPixel(x + i, y, c); }
  void drawFastVLine(int x, int y, int h, uint16_t c){ for (int i = 0; i < h; i++) drawPixel(x, y + i, c); }
  void drawLine(int x0, int y0, int x1, int y1, uint16_t c){
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1, dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1, e = dx + dy;
    for (;;){ drawPixel(x0, y0, c); if (x0 == x1 && y0 == y1) break; int e2 = 2 * e;
      if (e2 >= dy){ e += dy; x0 += sx; } if (e2 <= dx){ e += dx; y0 += sy; } }
  }
  void fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t c){
    int minx = std::min(x0, std::min(x1, x2)), maxx = std::max(x0, std::max(x1, x2));
    int miny = std::min(y0, std::min(y1, y2)), maxy = std::max(y0, std::max(y1, y2));
    for (int y = miny; y <= maxy; y++) for (int x = minx; x <= maxx; x++){
      auto side = [](int ax, int ay, int bx, int by, int px_, int py_){ return (bx - ax) * (py_ - ay) - (by - ay) * (px_ - ax); };
      int a = side(x0, y0, x1, y1, x, y), b = side(x1, y1, x2, y2, x, y), cc = side(x2, y2, x0, y0, x, y);
      if ((a >= 0 && b >= 0 && cc >= 0) || (a <= 0 && b <= 0 && cc <= 0)) drawPixel(x, y, c);
    }
  }
  void print(const char* s){
    for (; *s; s++){
      for (int col = 0; col < 5; col++){
        uint8_t line = font[(uint8_t)*s * 5 + col];
        for (int row = 0; row < 8; row++) if (line & (1 << row)) drawPixel(cx + col, cy + row, color);
      }
      cx += 6;
    }
  }
  void display(){}
  void save(const char* path){
    FILE* f = fopen(path, "w"); fprintf(f, "P1\n128 64\n");
    for (int y = 0; y < 64; y++){ for (int x = 0; x < 128; x++) fputs(px[y][x] ? "1 " : "0 ", f); fputc('\n', f); }
    fclose(f);
  }
};

int main(int argc, char** argv){
  const char* dir = argc > 1 ? argv[1] : ".";
  struct Case { const char* name; KickShapeInputs in; };
  Case cases[5];
  cases[0].name = "default";
  cases[1].name = "laser_tail_delay"; cases[1].in.shape = 127; cases[1].in.tailOn = true; cases[1].in.tailAmount = 90; cases[1].in.tailOffset = 80; cases[1].in.decay = 80;
  cases[2].name = "pitch_down_tmod";  cases[2].in.velocity = 10; cases[2].in.tailMod = 100; cases[2].in.decay = 90;
  cases[3].name = "round_long";       cases[3].in.shape = 0; cases[3].in.decay = 110; cases[3].in.sweepTime = 110;
  cases[4].name = "punchy_short";     cases[4].in.shape = 90; cases[4].in.decay = 30; cases[4].in.wave = 127; cases[4].in.sweepTime = 30;
  for (auto& c : cases){
    FakeOled d; drawKickShape(d, c.in, 1, 2);
    char path[512]; snprintf(path, sizeof(path), "%s/%s.pbm", dir, c.name); d.save(path);
    printf("%s\n", path);
  }
}
