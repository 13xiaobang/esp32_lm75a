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

static int lm75a_temp(int argc, char** argv)
{
    if(argc<2)
		return -1;
    if (strncmp(argv[1], "temp", 4) == 0) {
       
        lm75a_read_temperature();
        
    } else if(strncmp(argv[1], "init", 4) == 0) {
        lm75a_init();
    }
    else if(strncmp(argv[1], "dein", 4) == 0) {
        lm75a_deinit();
    }
    else if(strncmp(argv[1], "tos", 3) == 0) {
        lm75a_set_tos(atoi(argv[2]));
    }
    else if(strncmp(argv[1], "thys", 4) == 0) {
        lm75a_set_thys(atoi(argv[2]));
    }
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

