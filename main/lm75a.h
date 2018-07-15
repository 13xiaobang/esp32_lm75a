#ifndef LM75A_H
#define LM75A_H

int lm75a_read_temperature();
void lm75a_init();
void lm75a_deinit();
void lm75a_set_tos(int tos);
void lm75a_set_thys(int thys);

#endif
