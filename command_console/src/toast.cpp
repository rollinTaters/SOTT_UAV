#include "toast.hpp"

Toast::Toast(Vector2 pos, Vector2 res) {
  position = pos;
  resolution = res;
  bgColor = DARKGRAY;
}

void Toast::render() {
  DrawRectangleV(position, resolution, bgcolor);  
}

void Toast::newInfo(std::string msg, StatusCode code) {
  message = msg;
  scode = code;
}
