/*
  src/toast.hpp
  - A widget on top of the screen displays the responses of commands
  - Takes a message and StatusCode
 */

#pragma once
#include "raylib.h"
#include <string>

typedef enum Statuscode {
  SUCCESS,
  INFO,
  ERROR,
  STATUS_CODE_COUNT
} StatusCode;



class Toast {
public:  
  Toast(Vector2 pos, Vector2 res);
  void render();
  void newInfo(std::string msg, StatusCode code);

private:
  Vector2 position;
  Vector2 resolution;
  Color bgColor;
  Color fgColor;
  std::string message;
  StatusCode scode;

  float duration;
  
};
