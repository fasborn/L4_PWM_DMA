
#include "sk6812.h"


uint32_t BUF_DMA [ARRAY_LEN] = {0};

int leds[300] = {0};
uint8_t number_of_leds = 0;

void send_bite(uint8_t bite){
    if(bite){
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
        for(int i = 0; i<10; i++){}

        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
        for(int i = 0; i<8; i++){}
    }

    else{
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
        for(int i = 0; i<8; i++){}

        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
        for(int i = 0; i<10; i++){}
    }
}

void end_of_message(void){
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
    for(int i = 0; i<1500; i++){}
}

struct Color{
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	uint8_t warm;
};

struct Color int_to_rgbw(int color){
		
		struct Color rgbw = {0, 0, 0, 0};
		rgbw.green = 	(color & 0xFF000000)>>24;
    rgbw.red = 		(color & 0x00FF0000)>>16;
    rgbw.blue = 	(color & 0x0000FF00)>>8;
    rgbw.warm = 	color & 0xFF;

    return rgbw;
}

void send_pixel(int color){
	
		struct Color rgbw = {0, 0, 0, 0};
		rgbw = int_to_rgbw(color);

    for (uint8_t co = 0; co<8; co++){
        send_bite(rgbw.green>>(7-co));
    }
		
    for (uint8_t co = 0; co<8; co++){
        send_bite(rgbw.red>>(7-co));
    }
		
    for (uint8_t co = 0; co<8; co++){
        send_bite(rgbw.blue>>(7-co));
    }
		
    for (uint8_t co = 0; co<8; co++){
        send_bite(rgbw.warm>>(7-co));
    }
}

void send_word(uint8_t green, uint8_t red, uint8_t blue, uint8_t warm){

}

int rgbw_to_int(uint8_t green, uint8_t red, uint8_t blue, uint8_t warm){
    int rgbw_united = 0;
    rgbw_united = rgbw_united | ((int) green<<24);
    rgbw_united = rgbw_united | ((int) red<<16);
    rgbw_united = rgbw_united | ((int) blue<<8);
    rgbw_united = rgbw_united | ((int) warm);

    return rgbw_united;
}

void fill_color(uint8_t green, uint8_t red, uint8_t blue, uint8_t warm, uint8_t low, uint8_t high){
    
    int color = rgbw_to_int(green, red, blue, warm);

    for(uint8_t i = 0; i<low; i++){
      send_pixel(0);
    }

    for(uint8_t i = low; i<high; i++){
      send_pixel(color);
    }

    end_of_message();
}


void clear_strip(void){
    for(int i = 0; i<1500; i++){
      send_pixel(0);
    }

    end_of_message();
}

void fill_strip(void){
    for(uint8_t i = 0; i < number_of_leds; i++){
        send_pixel(leds[i]);
    }

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
    for (int i = 0; i<1500; i++){}
}

