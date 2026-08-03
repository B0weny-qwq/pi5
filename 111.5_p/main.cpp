// TASK5 reuses the complete TASK4 vision, motor, PDI and encoder-acceleration
// feedforward implementation. Only user-visible identity and evaluation time
// differ, so fixes cannot silently diverge between the two tasks.

#define BALL_TASK_LABEL "TASK5"
#define BALL_TASK_PREVIEW_NAME "ball2-task5-velocity"
#define BALL_TASK_EVALUATION_SECONDS 20.0
#define BALL_TASK_LOG_EVENT "task5_balance"
#define BALL_TASK_QUESTION_NUMBER 5
#define BALL_TASK_CONTROL_IPC_PORT 17305

#include "../111.4_p/main.cpp"
