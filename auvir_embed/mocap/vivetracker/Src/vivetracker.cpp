#include "vivetracker.h"
#include <cstring>

void nop() {}

void my_test() {
  int buf_src[10] = {1};
  int buf_dst[10] = {0};

  std::memcpy(buf_dst, buf_src, 10);

  nop();
}
