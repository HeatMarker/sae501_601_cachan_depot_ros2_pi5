#ifndef INC_APP_MAIN_H_
#define INC_APP_MAIN_H_

void app_config(void);
void app_loop(void);

extern float speed_speedo_data;
extern volatile uint32_t tim3_overflow_cnt;

#endif
