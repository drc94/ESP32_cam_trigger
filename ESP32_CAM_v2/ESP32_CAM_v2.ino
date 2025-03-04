#define DEBUG 0
#if DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_SERIAL(x) Serial.begin(x)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_SERIAL(x)
  #define DEBUG_PRINTF(...)
#endif

#include <esp_now.h>
#include "esp_log.h"
#include "esp_camera.h"
#include "sensor.h"

#define WAKEUP_INTERVAL 3000

static const char vernum[] = "v62.22";
char devname[30];

// https://sites.google.com/a/usapiens.com/opnode/time-zones  -- find your timezone here
String TIMEZONE = "GMT0BST,M3.5.0/01,M10.5.0/02";

#define Lots_of_Stats 1
#define blinking 0

int framesize;
int quality ;
int framesizeconfig ;
int qualityconfig ;
int buffersconfig ;
int avi_length ;            // how long a movie in seconds -- 1800 sec = 30 min
int frame_interval ;          // record at full speed
int speed_up_factor ;          // play at realtime

int MagicNumber = 12;                // change this number to reset the eprom in your esp32 for file numbers

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool configfile = false;
bool reboot_now = false;
bool restart_now = false;

String czone;

TaskHandle_t the_camera_loop_task;

static SemaphoreHandle_t wait_for_sd;
static SemaphoreHandle_t sd_go;
SemaphoreHandle_t baton;

long current_frame_time;
long last_frame_time;
int frame_buffer_size;

// https://github.com/espressif/esp32-camera/issues/182
#define fbs  1 // was 64 -- how many kb of static ram for psram -> sram buffer for sd write
uint8_t fb_record_static[fbs * 1024 + 20];

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

camera_fb_t * fb_curr = NULL;
camera_fb_t * fb_next = NULL;

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "soc/soc.h"
#include "esp_cpu.h" //#include "soc/cpu.h"
#include "soc/rtc_cntl_reg.h"

static esp_err_t cam_err;
float most_recent_fps = 0;
int most_recent_avg_framesize = 0;

uint8_t* fb_record;
uint8_t* fb_curr_record_buf;
uint8_t* fb_capture;

int fb_record_len;
int fb_curr_record_len;
int fb_capture_len;
long fb_record_time = 0;
long fb_curr_record_time = 0;
long fb_capture_time = 0;

int first = 1;
long frame_start = 0;
long frame_end = 0;
long frame_total = 0;
long frame_average = 0;
long loop_average = 0;
long loop_total = 0;
long total_frame_data = 0;
long last_frame_length = 0;
int done = 0;
long avi_start_time = 0;
long avi_end_time = 0;
int start_record = 0;

int we_are_already_stopped = 0;
long total_delay = 0;
long bytes_before_last_100_frames = 0;
long time_before_last_100_frames = 0;

long time_in_loop = 0;
long time_in_camera = 0;
long time_in_sd = 0;
long time_in_good = 0;
long time_total = 0;
long delay_wait_for_sd = 0;
long wait_for_cam = 0;
int very_high = 0;

bool do_the_ota = false;

int do_it_now = 0;
int gframe_cnt;
int gfblen;
int gj;
int  gmdelay;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  Avi Writer Stuff here


// MicroSD
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "FS.h"
#include <SD_MMC.h>

File logfile;
File avifile;
File idxfile;

char avi_file_name[100];
char file_to_edit[50] = "/JamCam0481.0007.avi"; //61.3

static int i = 0;
uint16_t frame_cnt = 0;
uint16_t remnant = 0;
uint32_t length = 0;
uint32_t startms;
uint32_t elapsedms;
uint32_t uVideoLen = 0;

int bad_jpg = 0;
int extend_jpg = 0;
int normal_jpg = 0;

int file_number = 0;
int file_group = 0;
long boot_time = 0;

long totalp;
long totalw;

#define BUFFSIZE 512

uint8_t buf[BUFFSIZE];

#define AVIOFFSET 240 // AVI main header length

unsigned long movi_size = 0;
unsigned long jpeg_size = 0;
unsigned long idx_offset = 0;

uint8_t zero_buf[4] = {0x00, 0x00, 0x00, 0x00};
uint8_t dc_buf[4] = {0x30, 0x30, 0x64, 0x63};    // "00dc"
uint8_t dc_and_zero_buf[8] = {0x30, 0x30, 0x64, 0x63, 0x00, 0x00, 0x00, 0x00};

uint8_t avi1_buf[4] = {0x41, 0x56, 0x49, 0x31};    // "AVI1"
uint8_t idx1_buf[4] = {0x69, 0x64, 0x78, 0x31};    // "idx1"


struct frameSizeStruct {
  uint8_t frameWidth[2];
  uint8_t frameHeight[2];
};

//  data structure from here https://github.com/s60sc/ESP32-CAM_MJPEG2SD/blob/master/avi.cpp, extended for ov5640
// must match https://github.com/espressif/esp32-camera/blob/b6a8297342ed728774036089f196d599f03ea367/driver/include/sensor.h#L87
// which changed in Nov 2024
static const frameSizeStruct frameSizeData[] = {
  {{0x60, 0x00}, {0x60, 0x00}}, // FRAMESIZE_96X96,    // 96x96    0 framesize
  {{0xA0, 0x00}, {0x78, 0x00}}, // FRAMESIZE_QQVGA,    // 160x120  1
  {{0x60, 0x00}, {0x60, 0x00}}, // FRAMESIZE_128X128   // 128x128  2
  {{0xB0, 0x00}, {0x90, 0x00}}, // FRAMESIZE_QCIF,     // 176x144  3
  {{0xF0, 0x00}, {0xB0, 0x00}}, // FRAMESIZE_HQVGA,    // 240x176  4
  {{0xF0, 0x00}, {0xF0, 0x00}}, // FRAMESIZE_240X240,  // 240x240  5
  {{0x40, 0x01}, {0xF0, 0x00}}, // FRAMESIZE_QVGA,     // 320x240  6
  {{0x40, 0x01}, {0xF0, 0x00}}, // FRAMESIZE_320X320,  // 320x320  7
  {{0x90, 0x01}, {0x28, 0x01}}, // FRAMESIZE_CIF,      // 400x296  8
  {{0xE0, 0x01}, {0x40, 0x01}}, // FRAMESIZE_HVGA,     // 480x320  9
  {{0x80, 0x02}, {0xE0, 0x01}}, // FRAMESIZE_VGA,      // 640x480  10
  //               38,400    61,440    153,600
  {{0x20, 0x03}, {0x58, 0x02}}, // FRAMESIZE_SVGA,     // 800x600   11
  {{0x00, 0x04}, {0x00, 0x03}}, // FRAMESIZE_XGA,      // 1024x768  12
  {{0x00, 0x05}, {0xD0, 0x02}}, // FRAMESIZE_HD,       // 1280x720  13
  {{0x00, 0x05}, {0x00, 0x04}}, // FRAMESIZE_SXGA,     // 1280x1024 14
  {{0x40, 0x06}, {0xB0, 0x04}}, // FRAMESIZE_UXGA,     // 1600x1200 15
  // 3MP Sensors
  {{0x80, 0x07}, {0x38, 0x04}}, // FRAMESIZE_FHD,      // 1920x1080 16
  {{0xD0, 0x02}, {0x00, 0x05}}, // FRAMESIZE_P_HD,     //  720x1280 17
  {{0x60, 0x03}, {0x00, 0x06}}, // FRAMESIZE_P_3MP,    //  864x1536 18
  {{0x00, 0x08}, {0x00, 0x06}}, // FRAMESIZE_QXGA,     // 2048x1536 19
  // 5MP Sensors
  {{0x00, 0x0A}, {0xA0, 0x05}}, // FRAMESIZE_QHD,      // 2560x1440 20
  {{0x00, 0x0A}, {0x40, 0x06}}, // FRAMESIZE_WQXGA,    // 2560x1600 21
  {{0x38, 0x04}, {0x80, 0x07}}, // FRAMESIZE_P_FHD,    // 1080x1920 22
  {{0x00, 0x0A}, {0x80, 0x07}}  // FRAMESIZE_QSXGA,    // 2560x1920 23

};

const int avi_header[AVIOFFSET] PROGMEM = {
  0x52, 0x49, 0x46, 0x46, 0xD8, 0x01, 0x0E, 0x00, 0x41, 0x56, 0x49, 0x20, 0x4C, 0x49, 0x53, 0x54,
  0xD0, 0x00, 0x00, 0x00, 0x68, 0x64, 0x72, 0x6C, 0x61, 0x76, 0x69, 0x68, 0x38, 0x00, 0x00, 0x00,
  0xA0, 0x86, 0x01, 0x00, 0x80, 0x66, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
  0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x49, 0x53, 0x54, 0x84, 0x00, 0x00, 0x00,
  0x73, 0x74, 0x72, 0x6C, 0x73, 0x74, 0x72, 0x68, 0x30, 0x00, 0x00, 0x00, 0x76, 0x69, 0x64, 0x73,
  0x4D, 0x4A, 0x50, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x74, 0x72, 0x66,
  0x28, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00,
  0x01, 0x00, 0x18, 0x00, 0x4D, 0x4A, 0x50, 0x47, 0x00, 0x84, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x4E, 0x46, 0x4F,
  0x10, 0x00, 0x00, 0x00, 0x6A, 0x61, 0x6D, 0x65, 0x73, 0x7A, 0x61, 0x68, 0x61, 0x72, 0x79, 0x20,
  0x76, 0x36, 0x32, 0x20, 0x4C, 0x49, 0x53, 0x54, 0x00, 0x01, 0x0E, 0x00, 0x6D, 0x6F, 0x76, 0x69,
};


//
// Writes an uint32_t in Big Endian at current file position
//
static void inline print_quartet(unsigned long i, File fd) {

  uint8_t y[4];
  y[0] = i % 0x100;
  y[1] = (i >> 8) % 0x100;
  y[2] = (i >> 16) % 0x100;
  y[3] = (i >> 24) % 0x100;
  size_t i1_err = fd.write(y , 4);
}

//
// Writes 2 uint32_t in Big Endian at current file position
//
static void inline print_2quartet(unsigned long i, unsigned long j, File fd) {

  uint8_t y[8];
  y[0] = i % 0x100;
  y[1] = (i >> 8) % 0x100;
  y[2] = (i >> 16) % 0x100;
  y[3] = (i >> 24) % 0x100;
  y[4] = j % 0x100;
  y[5] = (j >> 8) % 0x100;
  y[6] = (j >> 16) % 0x100;
  y[7] = (j >> 24) % 0x100;
  size_t i1_err = fd.write(y , 8);
}

//
// if we have no camera, or sd card, then flash rear led on and off to warn the human SOS - SOS
//
void major_fail() {

  DEBUG_PRINTLN(" ");
  logfile.close();

  for  (int i = 0;  i < 10; i++) {                 // 10 loops or about 100 seconds then reboot
    for (int j = 0; j < 3; j++) {
      digitalWrite(33, LOW);   delay(150);
      digitalWrite(33, HIGH);  delay(150);
    }
    delay(1000);

    for (int j = 0; j < 3; j++) {
      digitalWrite(33, LOW);  delay(500);
      digitalWrite(33, HIGH); delay(500);
    }
    delay(1000);
    DEBUG_PRINT("Major Fail  "); DEBUG_PRINT(i); DEBUG_PRINT(" / "); DEBUG_PRINTLN(10);
  }

  ESP.restart();
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//

static void config_camera() {

  camera_config_t config;

  //DEBUG_PRINTLN("config camera");

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;

  config.pixel_format = PIXFORMAT_JPEG;

  DEBUG_PRINTF("Frame config %d, quality config %d, buffers config %d\n", framesizeconfig, qualityconfig, buffersconfig);

  config.frame_size =  (framesize_t)framesize;
  config.jpeg_quality = quality;
  config.fb_count = buffersconfig;

  // https://github.com/espressif/esp32-camera/issues/357#issuecomment-1047086477
  config.grab_mode      = CAMERA_GRAB_LATEST; //61.92

  if (Lots_of_Stats) {
    DEBUG_PRINTF("Before camera config ...");
    DEBUG_PRINTF("Internal Total heap %d, internal Free Heap %d, ", ESP.getHeapSize(), ESP.getFreeHeap());
    DEBUG_PRINTF("SPIRam Total heap   %d, SPIRam Free Heap   %d\n", ESP.getPsramSize(), ESP.getFreePsram());
  }
  esp_err_t cam_err = ESP_FAIL;
  int attempt = 5;
  while (attempt && cam_err != ESP_OK) {
    cam_err = esp_camera_init(&config);
    if (cam_err != ESP_OK) {
      DEBUG_PRINTF("Camera init failed with error 0x%x", cam_err);
      digitalWrite(PWDN_GPIO_NUM, 1);
      delay(500);
      digitalWrite(PWDN_GPIO_NUM, 0); // power cycle the camera (OV2640)
      attempt--;
    }
  }

  if (Lots_of_Stats) {
    DEBUG_PRINTF("After  camera config ...");
    DEBUG_PRINTF("Internal Total heap %d, internal Free Heap %d, ", ESP.getHeapSize(), ESP.getFreeHeap());
    DEBUG_PRINTF("SPIRam Total heap   %d, SPIRam Free Heap   %d\n", ESP.getPsramSize(), ESP.getFreePsram());
  }

  if (cam_err != ESP_OK) {
    major_fail();
  }

  sensor_t * ss = esp_camera_sensor_get();

  DEBUG_PRINTF("\nCamera started correctly, Type is %x (hex) of 9650, 7725, 2640, 3660, 5640\n\n", ss->id.PID);

  if (ss->id.PID == OV5640_PID ) {
    //DEBUG_PRINTLN("56 - going mirror");
    ss->set_hmirror(ss, 1);        // 0 = disable , 1 = enable
  } else {
    ss->set_hmirror(ss, 0);        // 0 = disable , 1 = enable
  }

  ss->set_brightness(ss, 1);  //up the blightness just a bit
  ss->set_saturation(ss, -2); //lower the saturation

  int x = 0;
  delay(500);
  for (int j = 0; j < 30; j++) {
    camera_fb_t * fb = esp_camera_fb_get(); // get_good_jpeg();
    if (!fb) {
      DEBUG_PRINTLN("Camera Capture Failed");
    } else {
      if (j < 3 || j > 27) DEBUG_PRINTF("Pic %2d, len=%7d, at mem %X\n", j, fb->len, (long)fb->buf);
      x = fb->len;
      esp_camera_fb_return(fb);
      delay(30);
    }
  }
  frame_buffer_size  = (( (x * 2) / (16 * 1024) ) + 1) * 16 * 1024 ;

  DEBUG_PRINTF("Buffer size for %d is %d\n", x, frame_buffer_size);
  DEBUG_PRINTF("End of setup ...");
  DEBUG_PRINTF("Internal Total heap %d, internal Free Heap %d, ", ESP.getHeapSize(), ESP.getFreeHeap());
  DEBUG_PRINTF("SPIRam Total heap   %d, SPIRam Free Heap   %d\n", ESP.getPsramSize(), ESP.getFreePsram());
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//


static esp_err_t init_sdcard()
{

  int succ = SD_MMC.begin("/sdcard", true, false, BOARD_MAX_SDMMC_FREQ, 7);
  if (succ) {
    DEBUG_PRINTF("SD_MMC Begin: %d\n", succ);
    uint8_t cardType = SD_MMC.cardType();
    DEBUG_PRINT("SD_MMC Card Type: ");
    if (cardType == CARD_MMC) {
      DEBUG_PRINTLN("MMC");
    } else if (cardType == CARD_SD) {
      DEBUG_PRINTLN("SDSC");
    } else if (cardType == CARD_SDHC) {
      DEBUG_PRINTLN("SDHC");
    } else {
      DEBUG_PRINTLN("UNKNOWN");
    }

    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    DEBUG_PRINTF("SD_MMC Card Size: %lluMB\n", cardSize);

  } else {
    DEBUG_PRINTF("Failed to mount SD card VFAT filesystem. \n");
    DEBUG_PRINTLN("Do you have an SD Card installed?");
    DEBUG_PRINTLN("Check pin 12 and 13, not grounded, or grounded with 10k resistors!\n\n");
    major_fail();
  }

  return ESP_OK;
}

#include "config.h"

void read_config_file() {

  // if there is a config.txt, use it plus defaults
  // else use defaults, and create a config.txt

  // put a file "config.txt" onto SD card, to set parameters different from your hardcoded parameters
  // it should look like this - one paramter per line, in the correct order, followed by 2 spaces, and any comments you choose

  String junk;

  String cname ;
  int cframesize ;
  int cquality = 12 ;
  int cbuffersconfig = 4;
  int clength ;
  int cinterval ;
  int cspeedup ;
  String czone ;

  delay(500);

  File config_file = SD_MMC.open("/config.txt", "r");

  if (config_file) {
    DEBUG_PRINTLN("Opened config.txt from SD");
  } else {
    DEBUG_PRINTLN("Failed to open config.txt - writing a default");

    // lets make a simple.txt config file
    File new_simple = SD_MMC.open("/config.txt", "w");
    new_simple.print(config_txt);
    new_simple.close();

    file_group = 1;
    file_number = 1;

    do_eprom_write();

    config_file = SD_MMC.open("/config.txt", "r");
  }

  DEBUG_PRINTLN("Reading config.txt");
  cname = config_file.readStringUntil(' ');
  junk = config_file.readStringUntil('\n');
  cframesize = config_file.parseInt();
  junk = config_file.readStringUntil('\n');

  clength = config_file.parseInt();
  junk = config_file.readStringUntil('\n');
  cinterval = config_file.parseInt();
  junk = config_file.readStringUntil('\n');
  cspeedup = config_file.parseInt();
  junk = config_file.readStringUntil('\n');
  czone = config_file.readStringUntil(' ');
  junk = config_file.readStringUntil('\n');
  config_file.close();

  DEBUG_PRINTF("=========   Data from config.txt and defaults  =========\n");
  DEBUG_PRINTF("Name %s\n", cname); logfile.printf("Name %s\n", cname);
  DEBUG_PRINTF("Framesize %d\n", cframesize); logfile.printf("Framesize %d\n", cframesize);
  DEBUG_PRINTF("Quality %d\n", cquality); logfile.printf("Quality %d\n", cquality);
  DEBUG_PRINTF("Buffers config %d\n", cbuffersconfig); logfile.printf("Buffers config %d\n", cbuffersconfig);
  DEBUG_PRINTF("Length %d\n", clength); logfile.printf("Length %d\n", clength);
  DEBUG_PRINTF("Interval %d\n", cinterval); logfile.printf("Interval %d\n", cinterval);
  DEBUG_PRINTF("Speedup %d\n", cspeedup); logfile.printf("Speedup %d\n", cspeedup);

  DEBUG_PRINTF("Zone len %d, %s\n", czone.length(), czone.c_str()); //logfile.printf("Zone len %d, %s\n", czone.length(), czone);


  framesize = cframesize;
  quality = cquality;
  buffersconfig = cbuffersconfig;
  avi_length = clength;
  frame_interval = cinterval;
  speed_up_factor = cspeedup;
  configfile = true;
  TIMEZONE = czone;

  cname.toCharArray(devname, cname.length() + 1);

}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  delete_old_stuff() - delete oldest files to free diskspace
//

void listDir( const char * dirname, uint8_t levels) {

  DEBUG_PRINTF("Listing directory: %s\n", "/");

  File root = SD_MMC.open("/");
  if (!root) {
    DEBUG_PRINTLN("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    DEBUG_PRINTLN("Not a directory");
    return;
  }

  File filex = root.openNextFile();
  while (filex) {
    if (filex.isDirectory()) {
      DEBUG_PRINT("  DIR : ");
      DEBUG_PRINTLN(filex.name());
      if (levels) {
        listDir( filex.name(), levels - 1);
      }
    } else {
      DEBUG_PRINT("  FILE: ");
      DEBUG_PRINT(filex.name());
      DEBUG_PRINT("  SIZE: ");
      DEBUG_PRINTLN(filex.size());
    }
    filex = root.openNextFile();
  }
}
//#include <detail/mimetable.h>
#include <list>
#include <tuple>

void delete_old_stuff() {
  using namespace std;
  using records = tuple<String, String, size_t, time_t>;
  list<records> dirList;

  int card = SD_MMC.cardSize()  / (1024 * 1024);
  int total = SD_MMC.totalBytes()  / (1024 * 1024);
  int used = SD_MMC.usedBytes()  / (1024 * 1024);

  DEBUG_PRINTF("Card  space: %5dMB\n", card);  // %llu
  DEBUG_PRINTF("Total space: %5dMB\n", total);
  DEBUG_PRINTF("Used  space: %5dMB\n", used);

  //listDir( "/", 0);

  float full = 1.0 * used / total;
  if (full  <  0.8) {
    DEBUG_PRINTF("Nothing deleted, %.1f%% disk full\n", 100.0 * full);
  } else {
    DEBUG_PRINTF("Disk is %.1f%% full ... deleting ...\n", 100.0 * full);

    int x = millis();
    File xdir = SD_MMC.open("/");
    File xf = xdir.openNextFile();

    while (xf) {
      if (xf.isDirectory()) {
        File xdir_sub = xf.openNextFile();
        String directory = xf.name();
        while (xdir_sub) {
          String fn = "/" + directory + "/" + xdir_sub.name();
          dirList.emplace_back("", fn, xdir_sub.size(), xdir_sub.getLastWrite());
          DEBUG_PRINTF("Added Sub: "); DEBUG_PRINTLN(fn);
          xdir_sub = xf.openNextFile();
        }
        xdir_sub.close();
      } else {
        dirList.emplace_back("", xf.name(), xf.size(), xf.getLastWrite());
        DEBUG_PRINTF("Added: "); DEBUG_PRINTLN(xf.name());
      }
      xf = xdir.openNextFile();
    }
    xdir.close();

    dirList.sort([](const records & f, const records & l) {                                 // sort by date
      return get<3>(f) < get<3>(l);
      return false;
    });

    DEBUG_PRINTF("Sort files took %d ms\n", millis() - x);

    for ( auto& iter : dirList) {
      String fn = get<1>(iter);
      size_t fsize = get<2>(iter);
      int dotIndex = fn.lastIndexOf('.');
      if (dotIndex != -1) {
        String ext = fn.substring(dotIndex + 1);
        if (ext.equalsIgnoreCase("avi")) {
          DEBUG_PRINT("Oldest file is "); DEBUG_PRINT(fn); DEBUG_PRINTF(", size %d\n", fsize);
          deleteFolderOrFile(fn.c_str());

          used = used - (fsize  / (1024 * 1024));
          full = 1.0 * used / total ;
          DEBUG_PRINTLN(full);
          if (full < 0.7) break;
        }
      }
    }
  }
}

void deleteFolderOrFile(const char * val) {
  //DEBUG_PRINTF("Deleting : %s\n", val);
  File f = SD_MMC.open("/" + String(val));
  if (!f) {
    DEBUG_PRINTF("Failed to open %s\n", val);
    return;
  }

  if (f.isDirectory()) {
    File file = f.openNextFile();
    while (file) {
      if (file.isDirectory()) {
        DEBUG_PRINT("  DIR : ");
        DEBUG_PRINTLN(file.name());
      } else {
        DEBUG_PRINT("  FILE: ");
        DEBUG_PRINT(file.name());
        DEBUG_PRINT("  SIZE: ");
        DEBUG_PRINT(file.size());
        if (SD_MMC.remove(file.name())) {
          DEBUG_PRINTLN(" deleted.");
        } else {
          DEBUG_PRINTLN(" FAILED.");
        }
      }
      file = f.openNextFile();
    }
    f.close();
    //Remove the dir
    if (SD_MMC.rmdir("/" + String(val))) {
      DEBUG_PRINTF("Dir %s removed\n", val);
    } else {
      DEBUG_PRINTLN("Remove dir failed");
    }

  } else {
    //Remove the file
    if (SD_MMC.remove("/" + String(val))) {
      DEBUG_PRINTF("File %s deleted\n", val);
    } else {
      DEBUG_PRINTLN("Delete failed");
    }
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  get_good_jpeg()  - take a picture and make sure it has a good jpeg
//
camera_fb_t *  get_good_jpeg() {

  camera_fb_t * fb;

  long start;
  int failures = 0;

  do {
    int fblen = 0;
    int foundffd9 = 0;
    long bp = millis();
    long mstart = micros();

    fb = esp_camera_fb_get();
    if (!fb) {
      DEBUG_PRINTLN("Camera Capture Failed");
      failures++;
    } else {
      long mdelay = micros() - mstart;

      int get_fail = 0;

      totalp = totalp + millis() - bp;
      time_in_camera = totalp;

      fblen = fb->len;

      for (int j = 1; j <= 1025; j++) {
        if (fb->buf[fblen - j] != 0xD9) {
          // no d9, try next for
        } else {                                     //DEBUG_PRINTLN("Found a D9");
          if (fb->buf[fblen - j - 1] == 0xFF ) {     //DEBUG_PRINT("Found the FFD9, junk is "); DEBUG_PRINTLN(j);
            if (j == 1) {
              normal_jpg++;
            } else {
              extend_jpg++;
            }
            foundffd9 = 1;
            if (Lots_of_Stats) {
              if (j > 9000) {                // was 900             //  rarely happens - sometimes on 2640
                DEBUG_PRINT("Frame "); DEBUG_PRINT(frame_cnt); logfile.print("Frame "); logfile.print(frame_cnt);
                DEBUG_PRINT(", Len = "); DEBUG_PRINT(fblen); logfile.print(", Len = "); logfile.print(fblen);
                //DEBUG_PRINT(", Correct Len = "); DEBUG_PRINT(fblen - j + 1);
                DEBUG_PRINT(", Extra Bytes = "); DEBUG_PRINTLN( j - 1); logfile.print(", Extra Bytes = "); logfile.println( j - 1);
                logfile.flush();
              }

              if ( (frame_cnt % 1000 == 50) || (frame_cnt < 1000 && frame_cnt % 100 == 50)) {
                gframe_cnt = frame_cnt;
                gfblen = fblen;
                gj = j;
                gmdelay = mdelay;
                //DEBUG_PRINTF("Frame %6d, len %6d, extra  %4d, cam time %7d ", frame_cnt, fblen, j - 1, mdelay / 1000);
                //logfile.printf("Frame %6d, len %6d, extra  %4d, cam time %7d ", frame_cnt, fblen, j - 1, mdelay / 1000);
                do_it_now = 1;
              }
            }
            break;
          }
        }
      }

      if (!foundffd9) {
        bad_jpg++;
        DEBUG_PRINTF("Bad jpeg, Frame %d, Len = %d \n", frame_cnt, fblen);
        logfile.printf("Bad jpeg, Frame %d, Len = %d\n", frame_cnt, fblen);

        esp_camera_fb_return(fb);
        failures++;

      } else {
        break;
        // count up the useless bytes
      }
    }

  } while (failures < 10);   // normally leave the loop with a break()

  // if we get 10 bad frames in a row, then quality parameters are too high - set them lower (+5), and start new movie
  if (failures == 10) {
    DEBUG_PRINTF("10 failures");
    logfile.printf("10 failures");
    logfile.flush();

    sensor_t * ss = esp_camera_sensor_get();
    int qual = ss->status.quality ;
    ss->set_quality(ss, qual + 5);
    quality = qual + 5;
    DEBUG_PRINTF("\n\nDecreasing quality due to frame failures %d -> %d\n\n", qual, qual + 5);
    logfile.printf("\n\nDecreasing quality due to frame failures %d -> %d\n\n", qual, qual + 5);
    delay(1000);

    start_record = 0;

    esp_sleep_enable_timer_wakeup(WAKEUP_INTERVAL);
    esp_deep_sleep_start();
    //reboot_now = true;
  }
  return fb;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  eprom functions  - increment the file_group, so files are always unique
//

#include <EEPROM.h>

struct eprom_data {
  int eprom_good;
  int file_group;
};

void do_eprom_read() {

  eprom_data ed;

  EEPROM.begin(200);
  EEPROM.get(0, ed);

  if (ed.eprom_good == MagicNumber) {
    DEBUG_PRINTLN("Good settings in the EPROM ");
    file_group = ed.file_group;
    file_group++;
    DEBUG_PRINT("New File Group "); DEBUG_PRINTLN(file_group );
  } else {
    DEBUG_PRINTLN("No settings in EPROM - Starting with File Group 1 ");
    file_group = 1;
  }
  do_eprom_write();
  file_number = 1;
}

void do_eprom_write() {

  eprom_data ed;
  ed.eprom_good = MagicNumber;
  ed.file_group  = file_group;

  DEBUG_PRINTLN("Writing to EPROM ...");

  EEPROM.begin(200);
  EEPROM.put(0, ed);
  EEPROM.commit();
  EEPROM.end();
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Make the avi functions
//
//   start_avi() - open the file and write headers
//   another_pic_avi() - write one more frame of movie
//   end_avi() - write the final parameters and close the file


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// start_avi - open the files and write in headers
//

static void start_avi() {
  char the_directory[30];

  long start = millis();

  DEBUG_PRINTLN("Starting an avi ");
  sprintf(the_directory, "/%s%03d",  devname, file_group);
  SD_MMC.mkdir(the_directory);

  sprintf(avi_file_name, "/%s%03d/%s%03d.%03d.avi",  devname, file_group, devname, file_group, file_number);

  file_number++;

  avifile = SD_MMC.open(avi_file_name, "w");
  idxfile = SD_MMC.open("/idx.tmp", "w");

  if (avifile) {
    DEBUG_PRINTF("File open: %s\n", avi_file_name);
    logfile.printf("File open: %s\n", avi_file_name);
  }  else  {
    DEBUG_PRINTLN("Could not open file");
    major_fail();
  }

  if (idxfile)  {
    //DEBUG_PRINTF("File open: %s\n", "//idx.tmp");
  }  else  {
    DEBUG_PRINTLN("Could not open file /idx.tmp");
    major_fail();
  }

  for ( i = 0; i < AVIOFFSET; i++) {
    char ch = pgm_read_byte(&avi_header[i]);
    buf[i] = ch;
  }

  memcpy(buf + 0x40, frameSizeData[framesize].frameWidth, 2);
  memcpy(buf + 0xA8, frameSizeData[framesize].frameWidth, 2);
  memcpy(buf + 0x44, frameSizeData[framesize].frameHeight, 2);
  memcpy(buf + 0xAC, frameSizeData[framesize].frameHeight, 2);

  size_t err = avifile.write(buf, AVIOFFSET);

  uint8_t ex_fps = 1;
  if (frame_interval == 0) {
    if (framesize >= 11) {
      ex_fps = 12.5 * speed_up_factor ;;
    } else {
      ex_fps = 25.0 * speed_up_factor;
    }
  } else {
    ex_fps = round(1000.0 / frame_interval * speed_up_factor);
  }

  avifile.seek( 0x84 , SeekSet);
  print_quartet((int)ex_fps, avifile);

  avifile.seek( 0x30 , SeekSet);
  print_quartet(3, avifile);  // magic number 3 means frame count not written // 61.3

  avifile.seek( AVIOFFSET, SeekSet);

  DEBUG_PRINT(F("\nRecording "));
  DEBUG_PRINT(avi_length);
  DEBUG_PRINTLN(" seconds.");

  startms = millis();

  totalp = 0;
  totalw = 0;

  jpeg_size = 0;
  movi_size = 0;
  uVideoLen = 0;
  idx_offset = 4;

  bad_jpg = 0;
  extend_jpg = 0;
  normal_jpg = 0;

  time_in_loop = 0;
  time_in_camera = 0;
  time_in_sd = 0;
  time_in_good = 0;
  time_total = 0;
  delay_wait_for_sd = 0;
  wait_for_cam = 0;
  very_high = 0;

  time_in_sd += (millis() - start);

  logfile.flush();
  avifile.flush();

} // end of start avi

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  another_save_avi saves another frame to the avi file, uodates index
//           -- pass in a fb pointer to the frame to add
//

static void another_save_avi(uint8_t* fb_buf, int fblen ) {
  long start = millis();

  int fb_block_length;
  uint8_t* fb_block_start;

  jpeg_size = fblen;

  remnant = (4 - (jpeg_size & 0x00000003)) & 0x00000003;

  long bw = millis();
  long frame_write_start = millis();

  int block_delay[10];
  int block_num = 0;

  fb_record_static[0] = 0x30;       // "00dc"
  fb_record_static[1] = 0x30;
  fb_record_static[2] = 0x64;
  fb_record_static[3] = 0x63;

  int jpeg_size_rem = jpeg_size + remnant;

  fb_record_static[4] = jpeg_size_rem % 0x100;
  fb_record_static[5] = (jpeg_size_rem >> 8) % 0x100;
  fb_record_static[6] = (jpeg_size_rem >> 16) % 0x100;
  fb_record_static[7] = (jpeg_size_rem >> 24) % 0x100;

  fb_block_start = fb_buf;

  if (fblen > fbs * 1024 - 8 ) {                     // fbs is the size of frame buffer static
    fb_block_length = fbs * 1024;
    fblen = fblen - (fbs * 1024 - 8);
    memcpy(fb_record_static + 8, fb_block_start, fb_block_length - 8);
    fb_block_start = fb_block_start + fb_block_length - 8;

  } else {
    fb_block_length = fblen + 8  + remnant;
    memcpy(fb_record_static + 8, fb_block_start,  fblen);
    fblen = 0;
  }

  size_t err = avifile.write(fb_record_static, fb_block_length);

  if (err != fb_block_length) {
    start_record = 0;
    DEBUG_PRINT("Giving up - Error on avi write: err = "); DEBUG_PRINT(err);
    DEBUG_PRINT(" len = "); DEBUG_PRINTLN(fb_block_length);
    logfile.print("Giving up - Error on avi write: err = "); logfile.print(err);
    logfile.print(" len = "); logfile.println(fb_block_length);

    esp_sleep_enable_timer_wakeup(WAKEUP_INTERVAL);
    esp_deep_sleep_start();
  }

  if (block_num < 10) block_delay[block_num++] = millis() - bw;

  while (fblen > 0) {

    if (fblen > fbs * 1024) {
      fb_block_length = fbs * 1024;
      fblen = fblen - fb_block_length;
    } else {
      fb_block_length = fblen  + remnant;
      fblen = 0;
    }

    memcpy(fb_record_static, fb_block_start, fb_block_length);

    size_t err = avifile.write(fb_record_static,  fb_block_length);

    if (err != fb_block_length) {
      DEBUG_PRINT("Error on avi write: err = "); DEBUG_PRINT(err);
      DEBUG_PRINT(" len = "); DEBUG_PRINTLN(fb_block_length);
    }

    if (block_num < 10) block_delay[block_num++] = millis() - bw;

    fb_block_start = fb_block_start + fb_block_length;
    delay(0);
  }


  movi_size += jpeg_size;
  uVideoLen += jpeg_size;
  long frame_write_end = millis();

  print_2quartet(idx_offset, jpeg_size, idxfile);

  idx_offset = idx_offset + jpeg_size + remnant + 8;

  movi_size = movi_size + remnant;

  if ( do_it_now == 1 ) {  // && frame_cnt < 1011
    do_it_now = 0;
    DEBUG_PRINTF("Frame %6d, len %6d, extra  %4d, cam time %7d,  sd time %4d -- \n", gframe_cnt, gfblen, gj - 1, gmdelay / 1000, millis() - bw);
    logfile.printf("Frame % 6d, len % 6d, extra  % 4d, cam time % 7d,  sd time % 4d -- \n", gframe_cnt, gfblen, gj - 1, gmdelay / 1000, millis() - bw);
    logfile.flush();
  }

  totalw = totalw + millis() - bw;
  time_in_sd += (millis() - start);


  if ( (millis() - bw) > totalw / frame_cnt * 10) {
    unsigned long x = avifile.position();
    DEBUG_PRINTF("Frame %6d, sd time very high %4d >>> %4d -- pos %X, ",  frame_cnt, millis() - bw, (totalw / frame_cnt), x );
    logfile.printf("Frame %6d, sd time very high %4d >>> %4d -- pos %X, ",  frame_cnt, millis() - bw, (totalw / frame_cnt), x );
    very_high++;
    DEBUG_PRINTF("Block %d, delay %5d ... ", 0, block_delay[0]);
    for (int i = 1; i < block_num; i++) {
      DEBUG_PRINTF("Block %d, delay %5d ..., ", i, block_delay[i] - block_delay[i - 1]);
      logfile.printf("Block %d, delay %5d ..., ", i, block_delay[i] - block_delay[i - 1]);
    }
    DEBUG_PRINTLN(" ");
    logfile.println(" ");
  }
  avifile.flush();
  idxfile.flush();

} // end of another_pic_avi

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//  end_avi writes the index, and closes the files
//

static void end_avi() {

  long start = millis();

  unsigned long current_end = avifile.position();

  DEBUG_PRINTLN("End of avi - closing the files");
  logfile.println("End of avi - closing the files");

  if (frame_cnt <  5 ) {
    DEBUG_PRINTLN("Recording screwed up, less than 5 frames, forget index\n");
    idxfile.close();
    avifile.close();
    int xx = remove("/idx.tmp");
    int yy = remove(avi_file_name);

  } else {

    elapsedms = millis() - startms;

    float fRealFPS = (1000.0f * (float)frame_cnt) / ((float)elapsedms) * speed_up_factor;

    float fmicroseconds_per_frame = 1000000.0f / fRealFPS;
    uint8_t iAttainedFPS = round(fRealFPS) ;
    uint32_t us_per_frame = round(fmicroseconds_per_frame);

    //Modify the MJPEG header from the beginning of the file, overwriting various placeholders

    avifile.seek( 4 , SeekSet);
    print_quartet(movi_size + 240 + 16 * frame_cnt + 8 * frame_cnt, avifile);

    avifile.seek( 0x20 , SeekSet);
    print_quartet(us_per_frame, avifile);

    unsigned long max_bytes_per_sec = (1.0f * movi_size * iAttainedFPS) / frame_cnt;

    avifile.seek( 0x24 , SeekSet);
    print_quartet(max_bytes_per_sec, avifile);

    avifile.seek( 0x30 , SeekSet);
    print_quartet(frame_cnt, avifile);

    avifile.seek( 0x8c , SeekSet);
    print_quartet(frame_cnt, avifile);

    avifile.seek( 0x84 , SeekSet);
    print_quartet((int)iAttainedFPS, avifile);

    avifile.seek( 0xe8 , SeekSet);
    print_quartet(movi_size + frame_cnt * 8 + 4, avifile);

    DEBUG_PRINTLN(F("\n*** Video recorded and saved ***\n"));

    DEBUG_PRINTF("Recorded %5d frames in %5d seconds\n", frame_cnt, elapsedms / 1000);
    DEBUG_PRINTF("File size is %u bytes\n", movi_size + 12 * frame_cnt + 4);
    DEBUG_PRINTF("Adjusted FPS is %5.2f\n", fRealFPS);
    DEBUG_PRINTF("Max data rate is %lu bytes/s\n", max_bytes_per_sec);
    DEBUG_PRINTF("Frame duration is %d us\n", us_per_frame);
    DEBUG_PRINTF("Average frame length is %d bytes\n", uVideoLen / frame_cnt);
    DEBUG_PRINT("Average picture time (ms) "); DEBUG_PRINTLN( 1.0 * totalp / frame_cnt);
    DEBUG_PRINT("Average write time (ms)   "); DEBUG_PRINTLN( 1.0 * totalw / frame_cnt );
    //DEBUG_PRINT("Normal jpg % ");  DEBUG_PRINTLN( 100.0 * normal_jpg / frame_cnt, 1 );
    //DEBUG_PRINT("Extend jpg % ");  DEBUG_PRINTLN( 100.0 * extend_jpg / frame_cnt, 1 );
    //DEBUG_PRINT("Bad    jpg % ");  DEBUG_PRINTLN( 100.0 * bad_jpg / frame_cnt, 5 );
    DEBUG_PRINTF("Slow sd writes %d, %5.3f %% \n", very_high, 100.0 * very_high / frame_cnt, 5 );

    DEBUG_PRINTF("Writting the index, %d frames\n", frame_cnt);

    logfile.printf("Recorded %5d frames in %5d seconds\n", frame_cnt, elapsedms / 1000);
    logfile.printf("File size is %u bytes\n", movi_size + 12 * frame_cnt + 4);
    logfile.printf("Adjusted FPS is %5.2f\n", fRealFPS);
    logfile.printf("Max data rate is %lu bytes/s\n", max_bytes_per_sec);
    logfile.printf("Frame duration is %d us\n", us_per_frame);
    logfile.printf("Average frame length is %d bytes\n", uVideoLen / frame_cnt);
    logfile.print("Average picture time (ms) "); logfile.println( 1.0 * totalp / frame_cnt);
    logfile.print("Average write time (ms)   "); logfile.println( 1.0 * totalw / frame_cnt );
    logfile.print("Normal jpg % ");  logfile.println( 100.0 * normal_jpg / frame_cnt, 1 );
    logfile.print("Extend jpg % ");  logfile.println( 100.0 * extend_jpg / frame_cnt, 1 );
    logfile.print("Bad    jpg % ");  logfile.println( 100.0 * bad_jpg / frame_cnt, 5 );
    logfile.printf("Slow sd writes %d, %5.3f %% \n", very_high, 100.0 * very_high / frame_cnt, 5 );

    logfile.printf("Writting the index, %d frames\n", frame_cnt);

    avifile.seek( current_end , SeekSet);

    idxfile.close();

    size_t i1_err = avifile.write(idx1_buf, 4);

    print_quartet(frame_cnt * 16, avifile);

    idxfile = SD_MMC.open("/idx.tmp", "r");

    if (idxfile)  {
      //DEBUG_PRINTF("File open: %s\n", "//idx.tmp");
      //logfile.printf("File open: %s\n", "/idx.tmp");
    }  else  {
      DEBUG_PRINTLN("Could not open index file");
      logfile.println("Could not open index file");
      major_fail();
    }

    char * AteBytes;
    AteBytes = (char*) malloc (8);

    for (int i = 0; i < frame_cnt; i++) {
      size_t res = idxfile.readBytes( AteBytes, 8);
      size_t i1_err = avifile.write(dc_buf, 4);
      size_t i2_err = avifile.write(zero_buf, 4);
      size_t i3_err = avifile.write((uint8_t *)AteBytes, 8);
    }

    free(AteBytes);

    idxfile.close();
    avifile.close();

    int xx = SD_MMC.remove("/idx.tmp");
  }

  DEBUG_PRINTLN("---");  logfile.println("---");

  time_in_sd += (millis() - start);

  DEBUG_PRINTLN("");
  time_total = millis() - startms;
  DEBUG_PRINTF("waiting for cam %10dms, %4.1f%%\n", wait_for_cam , 100.0 * wait_for_cam  / time_total);
  DEBUG_PRINTF("Time in camera  %10dms, %4.1f%%\n", time_in_camera, 100.0 * time_in_camera / time_total);
  DEBUG_PRINTF("waiting for sd  %10dms, %4.1f%%\n", delay_wait_for_sd , 100.0 * delay_wait_for_sd  / time_total);
  DEBUG_PRINTF("Time in sd      %10dms, %4.1f%%\n", time_in_sd    , 100.0 * time_in_sd     / time_total);
  DEBUG_PRINTF("time total      %10dms, %4.1f%%\n", time_total    , 100.0 * time_total     / time_total);

  logfile.printf("waiting for cam %10dms, %4.1f%%\n", wait_for_cam , 100.0 * wait_for_cam  / time_total);
  logfile.printf("Time in camera  %10dms, %4.1f%%\n", time_in_camera, 100.0 * time_in_camera / time_total);
  logfile.printf("waiting for sd  %10dms, %4.1f%%\n", delay_wait_for_sd , 100.0 * delay_wait_for_sd  / time_total);
  logfile.printf("Time in sd      %10dms, %4.1f%%\n", time_in_sd    , 100.0 * time_in_sd     / time_total);
  logfile.printf("time total      %10dms, %4.1f%%\n", time_total    , 100.0 * time_total     / time_total);

  logfile.flush();

  if (file_number == 198) {
    reboot_now = true;
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include "time.h"
#include <WiFi.h>

typedef struct struct_message {
    int id;
    int state;
} struct_message;

struct_message receivedData;

RTC_DATA_ATTR static int active_camera = 0;

long signal_time = 0;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    DEBUG_PRINT("Received data from: ");
    for (int i = 0; i < 6; i++) {
        DEBUG_PRINTF("%02X", info->src_addr[i]);
        if (i < 5) DEBUG_PRINT(":");
    }
    DEBUG_PRINTLN();
    
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    
    DEBUG_PRINT("ID: ");
    DEBUG_PRINTLN(receivedData.id);
    DEBUG_PRINT("State: ");
    DEBUG_PRINTLN(receivedData.state);

    active_camera = 1;

    esp_sleep_enable_timer_wakeup(200);
    esp_deep_sleep_start();
}

void init_wifi() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(500);
    DEBUG_PRINT("MAC ADDRESS: ");
    DEBUG_PRINTLN(WiFi.macAddress());
    if (esp_now_init() != ESP_OK) {
      DEBUG_PRINTLN("ESP-NOW initialization error");
      return;
    }

    esp_now_register_recv_cb(OnDataRecv);
}

void the_camera_loop (void* pvParameter);
void delete_old_stuff();

time_t now;
struct tm timeinfo;
char localip[20];

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void initCC()
{
  do_eprom_read();
    
  // SD camera init
  DEBUG_PRINTLN("Mounting the SD card ...");
  esp_err_t card_err = init_sdcard();
  if (card_err != ESP_OK) {
    DEBUG_PRINTF("SD Card init failed with error 0x%x", card_err);
    major_fail();
    return;
  }

  DEBUG_PRINTLN("Try to get parameters from config.txt ...");

  read_config_file();

  DEBUG_PRINTLN("Setting up the camera ...");
  config_camera();

  DEBUG_PRINTLN("Checking SD for available space ...");
  delete_old_stuff();

  fb_record = (uint8_t*)ps_malloc(frame_buffer_size); // buffer to store a jpg in motion // needs to be larger for big frames from ov5640
  fb_curr_record_buf = (uint8_t*)ps_malloc(frame_buffer_size);
  fb_capture = (uint8_t*)ps_malloc(frame_buffer_size); // buffer to store a jpg in motion // needs to be larger for big frames from ov5640

  DEBUG_PRINTLN("Creating the_camera_loop_task");

  baton = xSemaphoreCreateMutex();

  // prio 6 - higher than the camera loop(), and the streaming
  xTaskCreatePinnedToCore( the_camera_loop, "the_camera_loop", 5000, NULL, 4, &the_camera_loop_task, 0); //soc14

  delay(100);

  DEBUG_PRINTLN("Checking SD for available space ...");
  delete_old_stuff();

  char logname[60];
  char the_directory[30];

  sprintf(the_directory, "/%s%03d",  devname, file_group);
  SD_MMC.mkdir(the_directory);

  sprintf(logname, "/%s%03d/%s%03d.999.txt",  devname, file_group, devname, file_group);
  DEBUG_PRINTF("Creating logfile %s\n",  logname);
  logfile = SD_MMC.open(logname, FILE_WRITE);

  if (!logfile) {
    DEBUG_PRINTLN("Failed to open logfile for writing");
  }
  
  boot_time = millis();

  DEBUG_PRINTLN("\n---  End of setup()  ---\n\n");
}


void setup() {
  digitalWrite(PWDN_GPIO_NUM, 1);
  DEBUG_SERIAL(115200);
  /*DEBUG_PRINTLN("\n\n---");

  pinMode(33, OUTPUT);             // little red led on back of chip
  digitalWrite(33, LOW);           // turn on the red LED on the back of chip

  pinMode(4, OUTPUT);               // Blinding Disk-Avtive Light
  digitalWrite(4, LOW);             // turn off

  //Serial.setDebugOutput(true);

  DEBUG_PRINTLN("                                    ");
  DEBUG_PRINTLN("-------------------------------------");
  DEBUG_PRINTF("ESP32-CAM-Video-Recorder-junior %s\n", vernum);
  DEBUG_PRINTLN("-------------------------------------");

  DEBUG_PRINT("setup, core ");  DEBUG_PRINT(xPortGetCoreID());
  DEBUG_PRINT(", priority = "); DEBUG_PRINTLN(uxTaskPriorityGet(NULL));

  esp_reset_reason_t reason = esp_reset_reason();

  logfile.print("--- reboot ------ because: ");
  DEBUG_PRINT("--- reboot ------ because: ");

  switch (reason) {
    case ESP_RST_UNKNOWN : DEBUG_PRINTLN("ESP_RST_UNKNOWN"); logfile.println("ESP_RST_UNKNOWN"); break;
    case ESP_RST_POWERON : DEBUG_PRINTLN("ESP_RST_POWERON"); logfile.println("ESP_RST_POWERON"); break;
    case ESP_RST_EXT : DEBUG_PRINTLN("ESP_RST_EXT"); logfile.println("ESP_RST_EXT"); break;
    case ESP_RST_SW : DEBUG_PRINTLN("ESP_RST_SW"); logfile.println("ESP_RST_SW"); break;
    case ESP_RST_PANIC : DEBUG_PRINTLN("ESP_RST_PANIC"); logfile.println("ESP_RST_PANIC"); break;
    case ESP_RST_INT_WDT : DEBUG_PRINTLN("ESP_RST_INT_WDT"); logfile.println("ESP_RST_INT_WDT"); break;
    case ESP_RST_TASK_WDT : DEBUG_PRINTLN("ESP_RST_TASK_WDT"); logfile.println("ESP_RST_TASK_WDT"); break;
    case ESP_RST_WDT : DEBUG_PRINTLN("ESP_RST_WDT"); logfile.println("ESP_RST_WDT"); break;
    case ESP_RST_DEEPSLEEP : DEBUG_PRINTLN("ESP_RST_DEEPSLEEP"); logfile.println("ESP_RST_DEEPSLEEP"); break;
    case ESP_RST_BROWNOUT : DEBUG_PRINTLN("ESP_RST_BROWNOUT"); logfile.println("ESP_RST_BROWNOUT"); break;
    case ESP_RST_SDIO : DEBUG_PRINTLN("ESP_RST_SDIO"); logfile.println("ESP_RST_SDIO"); break;
    default  : DEBUG_PRINTLN("Reset reason"); logfile.println("ESP ???"); break;
  }*/

  DEBUG_PRINTLN("Init communications...");

  if(active_camera)
  {
    digitalWrite(PWDN_GPIO_NUM, 0);
    signal_time = millis();
    start_record = 1;
    initCC();
    init_wifi();
  }
  else
  {
    init_wifi();
    esp_sleep_enable_timer_wakeup(WAKEUP_INTERVAL);
    delay(1000);
    esp_deep_sleep_start();
  }

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// the_camera_loop()
int delete_old_stuff_flag = 0;

void the_camera_loop (void* pvParameter) {

  DEBUG_PRINT("the camera loop, core ");  DEBUG_PRINT(xPortGetCoreID());
  DEBUG_PRINT(", priority = "); DEBUG_PRINTLN(uxTaskPriorityGet(NULL));

  frame_cnt = 0;

  start_record = 1;

  delay(500);

  while (1) {
    delay(1);

    // if (frame_cnt == 0 && start_record == 0)  // do nothing
    // if (frame_cnt == 0 && start_record == 1)  // start a movie
    // if (frame_cnt > 0 && start_record == 0)   // stop the movie
    // if (frame_cnt > 0 && start_record != 0)   // another frame

    ///////////////////  NOTHING TO DO //////////////////
    if ( (frame_cnt == 0 && start_record == 0)) {
      delay(100);

      ///////////////////  START A MOVIE  //////////////////
    } else if (frame_cnt == 0 && start_record == 1) {

      DEBUG_PRINTLN("Ready to start");

      we_are_already_stopped = 0;

      avi_start_time = millis();
      DEBUG_PRINTF("\nStart the avi ... at %d\n", avi_start_time);
      DEBUG_PRINTF("Framesize %d, quality %d, length %d seconds\n\n", framesize, quality, avi_length);
      logfile.printf("\nStart the avi ... at %d\n", avi_start_time);
      logfile.printf("Framesize %d, quality %d, length %d seconds\n\n", framesize, quality, avi_length);
      logfile.flush();

      //88 frame_cnt++;

      long wait_for_cam_start = millis();
      wait_for_cam += millis() - wait_for_cam_start;

      start_avi();

      wait_for_cam_start = millis();

      ///
      frame_cnt++;

      long delay_wait_for_sd_start = millis();

      delay_wait_for_sd += millis() - delay_wait_for_sd_start;

      fb_curr = get_good_jpeg();    //7

      fb_curr_record_len = fb_curr->len;
      memcpy(fb_curr_record_buf, fb_curr->buf, fb_curr->len);
      fb_curr_record_time = millis();

      xSemaphoreTake( baton, portMAX_DELAY );

      fb_record_len = fb_curr_record_len;
      memcpy(fb_record, fb_curr_record_buf, fb_curr_record_len);   // v59.5
      fb_record_time = fb_curr_record_time;
      xSemaphoreGive( baton );

      esp_camera_fb_return(fb_curr);  //7

      another_save_avi( fb_curr_record_buf, fb_curr_record_len );

      ///
      wait_for_cam += millis() - wait_for_cam_start;
      if (blinking) digitalWrite(33, frame_cnt % 2);                // blink

      ///////////////////  END THE MOVIE //////////////////
    } else if ( restart_now || reboot_now || (frame_cnt > 0 && start_record == 0) ||  millis() > (avi_start_time + avi_length * 1000)) { // end the avi

      DEBUG_PRINTLN("End the Avi");
      restart_now = false;

      if (blinking)  digitalWrite(33, frame_cnt % 2);

      end_avi();                                // end the movie

      if (blinking) digitalWrite(33, HIGH);          // light off

      delete_old_stuff_flag = 1;
      delay(50);

      avi_end_time = millis();

      float fps = 1.0 * frame_cnt / ((avi_end_time - avi_start_time) / 1000) ;

      DEBUG_PRINTF("End the avi at %d.  It was %d frames, %d ms at %.2f fps...\n", millis(), frame_cnt, avi_end_time, avi_end_time - avi_start_time, fps);
      logfile.printf("End the avi at %d.  It was %d frames, %d ms at %.2f fps...\n", millis(), frame_cnt, avi_end_time, avi_end_time - avi_start_time, fps);

      frame_cnt = 0;             // start recording again on the next loop

      if (millis() - signal_time < (avi_length - 2) * 1000)
      {
        start_record = 1;
        
        active_camera = 1;
      }
      else
      {
        start_record = 0;
        
        active_camera = 0;
        esp_sleep_enable_timer_wakeup(WAKEUP_INTERVAL);
        esp_deep_sleep_start();
      }

      ///////////////////  ANOTHER FRAME  //////////////////
    } else if (frame_cnt > 0 && start_record != 0) {  // another frame of the avi

      //DEBUG_PRINTLN("Another frame");

      current_frame_time = millis();
      if (current_frame_time - last_frame_time < frame_interval) {
        delay(frame_interval - (current_frame_time - last_frame_time));             // delay for timelapse
      }
      last_frame_time = millis();

      frame_cnt++;

      long delay_wait_for_sd_start = millis();
      delay_wait_for_sd += millis() - delay_wait_for_sd_start;

      fb_curr = get_good_jpeg();    //7

      fb_curr_record_len = fb_curr->len;
      memcpy(fb_curr_record_buf, fb_curr->buf, fb_curr->len);
      fb_curr_record_time = millis();

      xSemaphoreTake( baton, portMAX_DELAY );

      fb_record_len = fb_curr_record_len;
      memcpy(fb_record, fb_curr_record_buf, fb_curr_record_len);   // v59.5
      fb_record_time = fb_curr_record_time;
      xSemaphoreGive( baton );

      esp_camera_fb_return(fb_curr);  //7

      another_save_avi( fb_curr_record_buf, fb_curr_record_len );

      long wait_for_cam_start = millis();

      wait_for_cam += millis() - wait_for_cam_start;

      if (blinking) digitalWrite(33, frame_cnt % 2);

      if (frame_cnt % 100 == 10 ) {     // print some status every 100 frames
        if (frame_cnt == 10) {
          bytes_before_last_100_frames = movi_size;
          time_before_last_100_frames = millis();
          most_recent_fps = 0;
          most_recent_avg_framesize = 0;
        } else {

          most_recent_fps = 100.0 / ((millis() - time_before_last_100_frames) / 1000.0) ;
          most_recent_avg_framesize = (movi_size - bytes_before_last_100_frames) / 100;

          if ( (Lots_of_Stats && frame_cnt < 1011) || (Lots_of_Stats && frame_cnt % 1000 == 10)) {
            DEBUG_PRINTF("So far: %04d frames, in %6.1f seconds, for last 100 frames: avg frame size %6.1f kb, %.2f fps ...\n", frame_cnt, 0.001 * (millis() - avi_start_time), 1.0 / 1024  * most_recent_avg_framesize, most_recent_fps);
            logfile.printf("So far: %04d frames, in %6.1f seconds, for last 100 frames: avg frame size %6.1f kb, %.2f fps ...\n", frame_cnt, 0.001 * (millis() - avi_start_time), 1.0 / 1024  * most_recent_avg_framesize, most_recent_fps);
          }

          total_delay = 0;

          bytes_before_last_100_frames = movi_size;
          time_before_last_100_frames = millis();
        }
      }
    }
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// loop() - loop runs at low prio, so I had to move it to the task the_camera_loop at higher priority

void loop() {

}
