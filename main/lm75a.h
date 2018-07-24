#ifndef LM75A_H
#define LM75A_H

int lm75a_read_temperature(int show);
void lm75a_init();
void lm75a_deinit();
void lm75a_set_tos(int tos);
void lm75a_set_thys(int thys);
void lm75a_get_tos();
void lm75a_get_thys();
void lm75a_set_int(int en);
void lm75a_get_osio();
#endif
