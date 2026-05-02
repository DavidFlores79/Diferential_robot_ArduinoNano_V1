#pragma once

struct PID {
  float target;
  float integral;
  float lastError;
  long  lastCount;
  float speed;
  int   pwmOut;
};
