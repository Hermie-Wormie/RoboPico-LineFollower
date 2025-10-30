#include "queue.h"
#include "lwip/ip_addr.h"

// main.c (For TaskManager switching)
extern QueueHandle_t received_queue;
extern QueueHandle_t commands_queue;
extern TaskHandle_t TaskManager_T;
extern TaskHandle_t LED_T;
extern TaskHandle_t GPIO_T;
extern TaskHandle_t UDP_T;
extern TaskHandle_t Message_T;
extern TaskHandle_t Command_T;
extern TaskHandle_t Heartbeat_T;
extern TaskHandle_t Motor_T;
extern TaskHandle_t TaskManager_T;
extern TaskHandle_t LineFollowing_T;
extern TaskHandle_t Ultrasonic_T;
extern TaskHandle_t BarcodesPulse_T;
extern TaskHandle_t AutoTaskManager_T;
extern TaskHandle_t Station1_T;
extern TaskHandle_t TestHandle_1;
extern TaskHandle_t TestHandle_2;

// blink.c
void led_on();
void led_off();
void flash(int count, bool mode);
void blink(void *pvParameters);
void GPIO_blink(void *param);

// wifi.c
extern uint16_t total_packets_received;
extern ip_addr_t remote_ip;
extern ip_addr_t telemetry_ip;
void start_UDP_server_ap(void *pvParameters);
void start_UDP_server_hotspot(void *pvParameters);
void send_udp_packet(const char *data, const ip_addr_t *client_ip, uint16_t client_port);

// io_handler.c
#define BUZZER_PIN 18
#define CLOCK_FREQUENCY 125000000.0f
#define FREQUENCY 1000.0f
#define MESSAGE_BUFFER 16
#define MODE_SELECT_BUTTON 22
#define REMOTE_MODE 0xF1
#define AUTOMATIC_MODE 0xF2
#define STATION_1 0xF4
bool get_connection_mode(void);
void message_handler(void *pvParameters);
void heartbeat_task(void *pvParameters);
void buzzer(int count);
void task_manager(void *pvParameters);
void line_following_task(void *pvParameters);
void auto_task_switcher(void *pvParameters);

// interrupt.c
void setup_interrupts();
void enable_encoder_interrupts(void);
void disable_encoder_interrupts(void);
void enable_IR_interrupts(void);
void disable_IR_interrupts(void);

// infrared.c
void sample_ir_task();

// From encoder.c
extern volatile uint32_t pulse_width_L;
extern volatile uint32_t pulse_width_R;

// motor.c
void setup_gpio_motor();
void setup_pwm_motor(); 
void motor_task(void *params);
void process_motor_commands(void *params);
void encoder_debug_task(void *params);
void reset_motor();

// ultrasonic.c
void ultrasonic_task(void *pvParameters);
void set_ultrasonic_polldelay(uint16_t delay);
void create_semaphores();

// From barcodes.c
void barcode_width_processor(void *pvParameters);

// From station1.c
void station_1_task();

// Set 1 to print
#if 1
#define DEBUG_PRINT(fmt, args...) printf(fmt, ##args)
#else
#define DEBUG_PRINT(fmt, args...) // Nothing happens if DEBUG is 0
#endif