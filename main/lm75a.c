#if 0
/**
 * TEST CODE BRIEF
 *
 * This example will show you how to use I2C module by running two tasks on i2c bus:
 *
 * - read external i2c sensor, here we use a BH1750 light sensor(GY-30 module) for instance.
 * - Use one I2C port(master mode) to read or write the other I2C port(slave mode) on one ESP32 chip.
 *
 * Pin assignment:
 *
 * - slave :
 *    GPIO25 is assigned as the data signal of i2c slave port
 *    GPIO26 is assigned as the clock signal of i2c slave port
 * - master:
 *    GPIO18 is assigned as the data signal of i2c master port
 *    GPIO19 is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect GPIO18 with GPIO25
 * - connect GPIO19 with GPIO26
 * - connect sda/scl of sensor with GPIO18/GPIO19
 * - no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 *
 * Test items:
 *
 * - read the sensor data, if connected.
 * - i2c master(ESP32) will write data to i2c slave(ESP32).
 * - i2c master(ESP32) will read data from i2c slave(ESP32).
 */

#define DATA_LENGTH                        512              /*!<Data buffer length for test buffer*/
#define RW_TEST_LENGTH                     129              /*!<Data length for r/w test, any value from 0-DATA_LENGTH*/
#define DELAY_TIME_BETWEEN_ITEMS_MS        1234             /*!< delay time between different test items */

#define I2C_EXAMPLE_SLAVE_SCL_IO           26               /*!<gpio number for i2c slave clock  */
#define I2C_EXAMPLE_SLAVE_SDA_IO           25               /*!<gpio number for i2c slave data */
#define I2C_EXAMPLE_SLAVE_NUM              I2C_NUM_0        /*!<I2C port number for slave dev */
#define I2C_EXAMPLE_SLAVE_TX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave tx buffer size */
#define I2C_EXAMPLE_SLAVE_RX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave rx buffer size */


#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */

#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */

#define BH1750_SENSOR_ADDR                 0x23             /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START                   0x23             /*!< Command to set measure mode */



SemaphoreHandle_t print_mux = NULL;




/**
 * @brief test code to write esp-i2c-slave
 *
 * 1. set mode
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
static esp_err_t i2c_example_master_sensor_test(i2c_port_t i2c_num, uint8_t* data_h, uint8_t* data_l)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, BH1750_CMD_START, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_h, ACK_VAL);
    i2c_master_read_byte(cmd, data_l, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}



/**
 * @brief i2c slave initialization
 */
static void i2c_example_slave_init()
{
    int i2c_slave_port = I2C_EXAMPLE_SLAVE_NUM;
    i2c_config_t conf_slave;
    conf_slave.sda_io_num = I2C_EXAMPLE_SLAVE_SDA_IO;
    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.scl_io_num = I2C_EXAMPLE_SLAVE_SCL_IO;
    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.mode = I2C_MODE_SLAVE;
    conf_slave.slave.addr_10bit_en = 0;
    conf_slave.slave.slave_addr = ESP_SLAVE_ADDR;
    i2c_param_config(i2c_slave_port, &conf_slave);
    i2c_driver_install(i2c_slave_port, conf_slave.mode,
                       I2C_EXAMPLE_SLAVE_RX_BUF_LEN,
                       I2C_EXAMPLE_SLAVE_TX_BUF_LEN, 0);
}

/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t* buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if (( i + 1 ) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}

static void i2c_test_task(void* arg)
{
    int i = 0;
    int ret;
    uint32_t task_idx = (uint32_t) arg;
    uint8_t* data = (uint8_t*) malloc(DATA_LENGTH);
    uint8_t* data_wr = (uint8_t*) malloc(DATA_LENGTH);
    uint8_t* data_rd = (uint8_t*) malloc(DATA_LENGTH);
    uint8_t sensor_data_h, sensor_data_l;
    int cnt = 0;
    while (1) {
        printf("test cnt: %d\n", cnt++);
        ret = i2c_example_master_sensor_test( I2C_EXAMPLE_MASTER_NUM, &sensor_data_h, &sensor_data_l);
        xSemaphoreTake(print_mux, portMAX_DELAY);
        if(ret == ESP_ERR_TIMEOUT) {
            printf("I2C timeout\n");
        } else if(ret == ESP_OK) {
            printf("*******************\n");
            printf("TASK[%d]  MASTER READ SENSOR( BH1750 )\n", task_idx);
            printf("*******************\n");
            printf("data_h: %02x\n", sensor_data_h);
            printf("data_l: %02x\n", sensor_data_l);
            printf("sensor val: %f\n", (sensor_data_h << 8 | sensor_data_l) / 1.2);
        } else {
            printf("%s: No ack, sensor not connected...skip...\n", esp_err_to_name(ret));
        }
        xSemaphoreGive(print_mux);
        vTaskDelay(( DELAY_TIME_BETWEEN_ITEMS_MS * ( task_idx + 1 ) ) / portTICK_RATE_MS);
        //---------------------------------------------------
        for (i = 0; i < DATA_LENGTH; i++) {
            data[i] = i;
        }
        xSemaphoreTake(print_mux, portMAX_DELAY);
        size_t d_size = i2c_slave_write_buffer(I2C_EXAMPLE_SLAVE_NUM, data, RW_TEST_LENGTH, 1000 / portTICK_RATE_MS);
        if (d_size == 0) {
            printf("i2c slave tx buffer full\n");
            ret = i2c_example_master_read_slave(I2C_EXAMPLE_MASTER_NUM, data_rd, DATA_LENGTH);
        } else {
            ret = i2c_example_master_read_slave(I2C_EXAMPLE_MASTER_NUM, data_rd, RW_TEST_LENGTH);
        }

        if (ret == ESP_ERR_TIMEOUT) {
            printf("I2C timeout\n");
            printf("*********\n");
        } else if (ret == ESP_OK) {
            printf("*******************\n");
            printf("TASK[%d]  MASTER READ FROM SLAVE\n", task_idx);
            printf("*******************\n");
            printf("====TASK[%d] Slave buffer data ====\n", task_idx);
            disp_buf(data, d_size);
            printf("====TASK[%d] Master read ====\n", task_idx);
            disp_buf(data_rd, d_size);
        } else {
            printf("%s: Master read slave error, IO not connected...\n", esp_err_to_name(ret));
        }
        xSemaphoreGive(print_mux);
        vTaskDelay(( DELAY_TIME_BETWEEN_ITEMS_MS * ( task_idx + 1 ) ) / portTICK_RATE_MS);
        //---------------------------------------------------
        int size;
        for (i = 0; i < DATA_LENGTH; i++) {
            data_wr[i] = i + 10;
        }
        xSemaphoreTake(print_mux, portMAX_DELAY);
        //we need to fill the slave buffer so that master can read later
        ret = i2c_example_master_write_slave( I2C_EXAMPLE_MASTER_NUM, data_wr, RW_TEST_LENGTH);
        if (ret == ESP_OK) {
            size = i2c_slave_read_buffer( I2C_EXAMPLE_SLAVE_NUM, data, RW_TEST_LENGTH, 1000 / portTICK_RATE_MS);
        }
        if (ret == ESP_ERR_TIMEOUT) {
            printf("I2C timeout\n");
        } else if (ret == ESP_OK) {
            printf("*******************\n");
            printf("TASK[%d]  MASTER WRITE TO SLAVE\n", task_idx);
            printf("*******************\n");
            printf("----TASK[%d] Master write ----\n", task_idx);
            disp_buf(data_wr, RW_TEST_LENGTH);
            printf("----TASK[%d] Slave read: [%d] bytes ----\n", task_idx, size);
            disp_buf(data, size);
        } else {
            printf("TASK[%d] %s: Master write slave error, IO not connected....\n", task_idx, esp_err_to_name(ret));
        }
        xSemaphoreGive(print_mux);
        vTaskDelay(( DELAY_TIME_BETWEEN_ITEMS_MS * ( task_idx + 1 ) ) / portTICK_RATE_MS);
    }
}
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"


#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_SCL_IO          19               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO          18               /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define LM75A_SLAVE_ADDR                     0x48             /*!< LM75A slave address, you can set any 7bit value */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t* data_rd, size_t size)
{
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
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t* data_wr, size_t size)
{
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
static void i2c_master_init()
{
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

int lm75a_read_temperature()
{
    uint8_t  buf[2];
    float tmp;
    buf[0] = 0;
    i2c_master_write_slave(I2C_MASTER_NUM, buf, 1);
    i2c_master_read_slave(I2C_MASTER_NUM, buf, 2);
    tmp = buf[0];
    if(buf[1]& 128)
            tmp+=0.5;
    printf("data_rd=%.1f\n", tmp);
    return 0;
}

#define GPIO_INPUT_IO_0     4
#define GPIO_OUTPUT_IO_0 2
#define GPIO_OUTPUT_PIN_SEL (1ULL<<GPIO_OUTPUT_IO_0)
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)
#define ESP_INTR_FLAG_DEFAULT 0


static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static int gpio_int_task_enable = 0;
static TaskHandle_t gpio_int_task_handle = NULL;
static void gpio_int_task(void* arg)
{
    uint32_t io_num;
    gpio_int_task_enable = 1;
    while(gpio_int_task_enable) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
           
            //read temperature to clean int;
            if(io_num == GPIO_INPUT_IO_0) {
                printf("GPIO[%d] intr, val: %d\n\n", io_num, gpio_get_level(io_num));
                lm75a_read_temperature();
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

void init_os_gpio()
{
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

static void deinit_os_gpio()
{
    printf("deinit_os_gpio!\n");

    if(gpio_int_task_handle) {
        gpio_isr_handler_remove(GPIO_INPUT_IO_0);
        gpio_uninstall_isr_service();
        gpio_int_task_enable = 0;
        int io = 0;
        xQueueSend(gpio_evt_queue, &io , 0); // send a fake signal to quit task.
    }
}

static void lm75a_vcc_enable()
{
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
    gpio_set_pull_mode(GPIO_INPUT_IO_0, GPIO_PULLUP_ONLY);
    gpio_config(&io_conf);
    
    gpio_set_level(GPIO_OUTPUT_IO_0, 1);
}

static void lm75a_vcc_disable()
{
    gpio_set_level(GPIO_OUTPUT_IO_0, 0);
}

void lm75a_init()
{
    lm75a_vcc_enable();
    i2c_master_init();
}

void lm75a_deinit()
{
    deinit_os_gpio();
    i2c_driver_delete(I2C_MASTER_NUM);
    lm75a_vcc_disable();
}

void lm75a_set_tos(int tos)
{
    uint8_t buf[4];
    printf("lm75a_set_tos: %d\n" ,tos);
    // set Tos:
    buf[0] = 0x3;
    buf[1] = (tos&0xff);
    buf[2] = 0;
    i2c_master_write_slave(I2C_MASTER_NUM, buf, 3);
}

void lm75a_set_thys(int thys)
{
    uint8_t buf[4];
    printf("lm75a_set_thys: %d\n" ,thys);
    // set Thyst:
    buf[0] = 0x2;
    buf[1] = (thys&0xff);
    buf[2] = 0;
    i2c_master_write_slave(I2C_MASTER_NUM, buf, 3);
}

void lm75a_get_tos()
{
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

void lm75a_get_thys()
{
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

void lm75a_set_int(int en)
{
    uint8_t buf[2];
    en = !!en;
    if(en) {
        init_os_gpio();
        printf("lm75a_set_int: %d\n" , en);
        buf[0] = 0x1;
        buf[1] = (1<<1); // D1 set to 1;
        i2c_master_write_slave(I2C_MASTER_NUM, buf, 2);
    }
    else {
        printf("lm75a_set_int: %d\n" ,en);
        deinit_os_gpio();
        buf[0] = 0x1;
        buf[1] = 0;
        i2c_master_write_slave(I2C_MASTER_NUM, buf, 2);
    }
    
}