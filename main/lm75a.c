#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SCL_IO           19               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           18               /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ          400000           /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define LM75A_SLAVE_ADDR            0x48             /*!< LM75A slave address, you can set any 7bit value */
#define ACK_VAL                     0x0              /*!< I2C ack value */
#define NACK_VAL                    0x1              /*!< I2C nack value */
#define WRITE_BIT                   I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                    I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS               0x0              /*!< I2C master will not check ack from slave */

#define GPIO_INPUT_IO_0 4
#define GPIO_OUTPUT_IO_0 2
#define GPIO_OUTPUT_PIN_SEL (1ULL<<GPIO_OUTPUT_IO_0)
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)
#define ESP_INTR_FLAG_DEFAULT 0
static xQueueHandle gpio_evt_queue = NULL;
static int gpio_int_task_enable = 0;
static TaskHandle_t gpio_int_task_handle = NULL;

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t* data_rd, size_t size) {
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( LM75A_SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t* data_wr, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( LM75A_SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init() {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);
}

int lm75a_read_temperature(int show) {
    uint8_t  buf[2];
    float tmp;
    buf[0] = 0;
    i2c_master_write_slave(I2C_MASTER_NUM, buf, 1);
    i2c_master_read_slave(I2C_MASTER_NUM, buf, 2);
    tmp = buf[0];
    if(buf[1]& 128)
        tmp+=0.5;
    if(show)
        printf("lm75a_read_temperature=%.1f\n", tmp);
    return 0;
}

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_int_task(void* arg) {
    uint32_t io_num;
    gpio_int_task_enable = 1;
    while(gpio_int_task_enable) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {

            //read temperature to clean int;
            if(io_num == GPIO_INPUT_IO_0) {
                printf("GPIO[%d] intr, val: %d\n\n", io_num, gpio_get_level(io_num));
                lm75a_read_temperature(0); //read to clean interrupt.
            }
        }
    }
    printf("quit gpio_int_task\n");
    if(gpio_evt_queue) {
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue =NULL;
    }
    gpio_int_task_handle = NULL;
    vTaskDelete(NULL);

}

void init_os_gpio() {
    printf("init_os_gpio!\n");

    if(gpio_evt_queue == NULL)
        gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    if(gpio_int_task_handle==NULL) {
        xTaskCreate(gpio_int_task, "gpio_int_task", 2048, NULL, 10, &gpio_int_task_handle);
        //install gpio isr service
        gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
        //hook isr handler for specific gpio pin again
        gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    }
}

static void deinit_os_gpio() {
    printf("deinit_os_gpio!\n");

    if(gpio_int_task_handle) {
        gpio_isr_handler_remove(GPIO_INPUT_IO_0);
        gpio_uninstall_isr_service();
        gpio_int_task_enable = 0;
        int io = 0;
        xQueueSend(gpio_evt_queue, &io , 0); // send a fake signal to quit task.
    }
}

static void lm75a_vcc_enable() {
    gpio_config_t io_conf;
    //enable output for vcc
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    //enable input for interrupt
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;//GPIO_PIN_INTR_ANYEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_set_pull_mode(GPIO_INPUT_IO_0, GPIO_FLOATING);
    gpio_config(&io_conf);
    gpio_set_level(GPIO_OUTPUT_IO_0, 1);
}

static void lm75a_vcc_disable() {
    gpio_set_level(GPIO_OUTPUT_IO_0, 0);
}

void lm75a_init() {
    lm75a_vcc_enable();
    i2c_master_init();
}

void lm75a_deinit() {
    deinit_os_gpio();
    i2c_driver_delete(I2C_MASTER_NUM);
    lm75a_vcc_disable();
}

void lm75a_set_tos(int tos) {
    uint8_t buf[4];
    printf("lm75a_set_tos: %d\n" ,tos);
    // set Tos:
    buf[0] = 0x3;
    buf[1] = (tos&0xff);
    buf[2] = 0;
    i2c_master_write_slave(I2C_MASTER_NUM, buf, 3);
}

void lm75a_set_thys(int thys) {
    uint8_t buf[4];
    printf("lm75a_set_thys: %d\n" ,thys);
    // set Thyst:
    buf[0] = 0x2;
    buf[1] = (thys&0xff);
    buf[2] = 0;
    i2c_master_write_slave(I2C_MASTER_NUM, buf, 3);
}

void lm75a_get_tos() {
    uint8_t buf[4];
    float tmp;
    buf[0] = 0x3;
    i2c_master_write_slave(I2C_MASTER_NUM, buf, 1);
    i2c_master_read_slave(I2C_MASTER_NUM, buf, 2);
    tmp = buf[0];
    if(buf[1]& 128)
        tmp+=0.5;

    printf("lm75a_get_tos: %.1f\n" , tmp);
}

void lm75a_get_thys() {
    uint8_t buf[4];
    float tmp;
    buf[0] = 0x2;
    i2c_master_write_slave(I2C_MASTER_NUM, buf, 1);
    i2c_master_read_slave(I2C_MASTER_NUM, buf, 2);
    tmp = buf[0];
    if(buf[1]& 128)
        tmp+=0.5;

    printf("lm75a_get_thys: %.1f\n" , tmp);
}

void lm75a_set_int(int en) {
    uint8_t buf[2];
    en = !!en;
    if(en) {
        printf("lm75a_set_int: %d\n" , en);
        buf[0] = 0x1;
        buf[1] = (1<<1); // D1 set to 1;
        i2c_master_write_slave(I2C_MASTER_NUM, buf, 2);
        i2c_master_read_slave(I2C_MASTER_NUM, buf, 2); //do one time read to clean interrupt before enter interrupt mode.
        gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_NEGEDGE);
        init_os_gpio();
    } else {
        printf("lm75a_set_int: %d\n" ,en);
        deinit_os_gpio();
        buf[0] = 0x1;
        buf[1] = 0;
        i2c_master_write_slave(I2C_MASTER_NUM, buf, 2);
        i2c_master_read_slave(I2C_MASTER_NUM, buf, 2); //do one time read to clean interrupt before enter interrupt mode.
    }

}

void lm75a_get_osio() {
    printf("os_io: %d\n",gpio_get_level(GPIO_INPUT_IO_0));
}
