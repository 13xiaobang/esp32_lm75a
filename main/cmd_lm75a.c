#include "lm75a.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "cmd_decl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_event_loop.h"

#define TEMP_READ "temp"
#define TEMP_INIT "init"
#define TEMP_DEINIT "dein"
#define TEMP_SET_TOS "set_tos"
#define TEMP_SET_THYS "set_thys"
#define TEMP_GET_TOS "get_tos"
#define TEMP_GET_THYS "get_thys"
#define TEMP_SET_INTERRUPT "set_int"

static void local_help() {
    printf("lm75a cmd:\n\
        lm75a init: init lm75a dev and i2c;\n\
        lm75a deinit: deinit lm75a dev and i2c;\n\
        lm75a temp: read temperature;\n\
        lm75a set_tos: set Tos temperature;\n\
        lm75a get_tos: get Tos temperature;\n\
        lm75a set_thys: set Thyst temperature;\n\
        lm75a get_thys: get Thyst temperature;\n\
        lm75a set_int: 1/0:  enable/disable the interrupt mode;\n");
}
static int lm75a_temp(int argc, char** argv)
{
    if(argc<2)
		return -1;
    if (strncmp(argv[1], TEMP_READ,  strlen(TEMP_READ)) == 0) {
        lm75a_read_temperature();
    } else if(strncmp(argv[1], TEMP_INIT, strlen(TEMP_INIT)) == 0) {
        lm75a_init();
     } else if(strncmp(argv[1], TEMP_DEINIT, strlen(TEMP_DEINIT)) == 0) {
        lm75a_deinit();
    } else if(strncmp(argv[1], TEMP_SET_TOS, strlen(TEMP_SET_TOS)) == 0) {
        lm75a_set_tos(atoi(argv[2]));
    } else if(strncmp(argv[1], TEMP_SET_THYS, strlen(TEMP_SET_THYS)) == 0) {
        lm75a_set_thys(atoi(argv[2]));
    } else if(strncmp(argv[1], TEMP_GET_TOS, strlen(TEMP_GET_TOS)) == 0) {
        lm75a_get_tos();
    } else if(strncmp(argv[1], TEMP_GET_THYS, strlen(TEMP_GET_THYS)) == 0) {
        lm75a_get_thys();
    }  else if(strncmp(argv[1], TEMP_SET_INTERRUPT, strlen(TEMP_SET_INTERRUPT)) == 0) {
        lm75a_set_int(atoi(argv[2]));
    } else
       local_help();
    
    
    return 0;
}

void register_lm75a_cmd()
{
    const esp_console_cmd_t cmd = {
        .command = "lm75a",
        .help = "lm75a cmd:",
        .hint = NULL,
        .func = &lm75a_temp,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

