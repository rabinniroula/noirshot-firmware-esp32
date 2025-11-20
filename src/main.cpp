#include <Arduino.h>
#include "esp_camera.h"

// Select camera model
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#include "camera_pins.h"

void setup()
{
	Serial.begin(115200, SERIAL_8N1, 1, 3);
	// Serial.println();

	camera_config_t config;
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
	config.pixel_format = PIXFORMAT_GRAYSCALE;

	if (psramFound())
	{
		Serial.printf("PSRAM found");
		config.frame_size = FRAMESIZE_QVGA;
		config.jpeg_quality = 12;
		config.fb_count = 3;
	}
	else
	{
		Serial.printf("PSRAM Not found");

		config.frame_size = FRAMESIZE_QVGA;
		config.jpeg_quality = 12;
		config.fb_count = 1;
	}

	delay(1000);

	esp_err_t err = esp_camera_init(&config);
	if (err != ESP_OK)
	{
		Serial.printf("Camera init failed with error 0x%x", err);
		return;
	}
}

void clustered_dot_v1_ordered_dither(
    const uint8_t *src, uint8_t *dst,
    int width, int height
) {
    memset(dst, 0, (width * height + 7) / 8); // clear output buffer
    static const uint8_t bayer4x4[4][4] = {
        { 15, 135, 45, 165 },
        { 195, 75, 225, 105 },
        { 60, 180, 30, 150 },
        { 240, 120, 210, 90 }
    };
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int i = y * width + x;
            uint8_t pixel = src[i];
            // get threshold from Bayer matrix
            uint8_t threshold = bayer4x4[y % 4][x % 4];
            // Inverted logic: Compare pixel to threshold (white becomes black, black becomes white)
            int bit = pixel <= threshold ? 1 : 0;
            if (bit) {
                dst[i / 8] |= (1 << (7 - (i % 8))); // MSB-first bit packing
            }
        }
    }
}


uint8_t dithered[9600]; // (320 * 240 + 7) / 8 = 9600 bytes

void loop() {
    if (Serial.available()) {
        if (Serial.readStringUntil('\n') == "gimme") {
            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb) {
                Serial.println("Camera capture failed");
                return;
            }
            
            clustered_dot_v1_ordered_dither(fb->buf, dithered, 320, 240);
            
            // === MARKER START ===
            // Serial.println("===IMG_START===");
            
            // ESC @ (Initialize) and center align
            Serial.write("\x1B\x40", 2); // ESC @
            Serial.write("\x1B\x61\x01", 3); // ESC a 1 (center align)
            
            // Process image in 24-pixel high strips
            for (int y = 0; y < 240; y += 24) {
                // ESC * m nL nH (bit image command)
                Serial.write("\x1B\x2A\x21", 3); // ESC * 33 (24-dot double-density)
                uint8_t nL = 320 & 0xFF;
                uint8_t nH = 320 >> 8;
                Serial.write(nL);
                Serial.write(nH);
                
                // For each column (x position)
                for (int x = 0; x < 320; x++) {
                    uint8_t bytes[3] = {0, 0, 0};
                    
                    // Extract 24 vertical pixels for this column
                    for (int bit = 0; bit < 24; bit++) {
                        int yy = y + bit;
                        if (yy >= 240) continue;
                        
                        // Calculate position in dithered buffer (matches your working code)
                        int byte_index = yy * (320 / 8) + (x / 8);
                        int bit_index = 7 - (x % 8);
                        
                        // Extract pixel value
                        if ((dithered[byte_index] >> bit_index) & 1) {
                            bytes[bit / 8] |= (1 << (7 - (bit % 8)));
                        }
                    }
                    
                    Serial.write(bytes, 3);
                }
                
                Serial.write("\n"); // Line feed after each strip
            }
            
            // Cut command
            Serial.write("\x1D\x56\x00", 3);
            
            // === MARKER END ===
            char marker_end[] = "===IMG_END===";
            Serial.print(marker_end);
            
            esp_camera_fb_return(fb);
        }
    }
}