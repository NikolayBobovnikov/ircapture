#include <iostream>

void clearscreen() {
#ifdef __linux
  std::system("clear");
#elif WIN32
  std::system("cls");
#endif
}

int main() { return 0; }
