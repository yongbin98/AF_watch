
#include "st7789/st7789.h"
#include <string.h>
#include <stdlib.h>

#ifdef USE_DMA
uint16_t DMA_MIN_SIZE = 16;
/* If you're using DMA, then u need a "framebuffer" to store datas to be displayed.
 * If your MCU don't have enough RAM, please avoid using DMA(or set 5 to 1).
 * And if your MCU have enough RAM(even larger than full-frame size),
 * Then you can specify the framebuffer size to the full resolution below.
 */
 #define HOR_LEN 	5	//	Alse mind the resolution of your screen!
uint16_t disp_buf[ST7789_WIDTH * HOR_LEN];
#endif

/**
 * @brief Write command to ST7789 controller
 * @param cmd -> command to write
 * @return none
 */
static void ST7789_WriteCommand(uint8_t cmd)
{
	ST7789_Select();
	ST7789_DC_Clr();
	HAL_SPI_Transmit(&ST7789_SPI_PORT, &cmd, sizeof(cmd), HAL_MAX_DELAY);
	while(HAL_SPI_GetState(&ST7789_SPI_PORT) != HAL_SPI_STATE_READY){}
	HAL_SPI_Transmit_DMA(&ST7789_SPI_PORT, &cmd, sizeof(cmd));
	while(HAL_SPI_GetState(&ST7789_SPI_PORT) != HAL_SPI_STATE_READY){}
	ST7789_UnSelect();
}

static void ST7789_WriteSmallData(uint8_t data)
{
	ST7789_Select();
	ST7789_DC_Set();
	//HAL_SPI_Transmit(&ST7789_SPI_PORT, &data, sizeof(data), HAL_MAX_DELAY);
	HAL_SPI_Transmit_DMA(&ST7789_SPI_PORT, &data, sizeof(data));
	while(HAL_SPI_GetState(&ST7789_SPI_PORT) != HAL_SPI_STATE_READY){}
	ST7789_UnSelect();
}

/**
 * @brief Write data to ST7789 controller
 * @param buff -> pointer of data buffer
 * @param buff_size -> size of the data buffer
 * @return none
 */
static void ST7789_WriteData(uint8_t *buff, size_t buff_size)
{
	ST7789_Select();
	ST7789_DC_Set();

	// split data in small chunks because HAL can't send more than 64K at once

	while (buff_size > 0) {
		uint16_t chunk_size = buff_size > 65535 ? 65535 : buff_size;
		#ifdef USE_DMA
			if (DMA_MIN_SIZE <= buff_size)
			{
				HAL_SPI_Transmit_DMA(&ST7789_SPI_PORT, buff, chunk_size);
				while (ST7789_SPI_PORT.hdmatx->State != HAL_DMA_STATE_READY);
			}
			else
				HAL_SPI_Transmit(&ST7789_SPI_PORT, buff, chunk_size, HAL_MAX_DELAY);
		#else
			HAL_SPI_Transmit(&ST7789_SPI_PORT, buff, chunk_size, HAL_MAX_DELAY);
		#endif
		buff += chunk_size;
		buff_size -= chunk_size;
	}

	ST7789_UnSelect();
}


/**
 * @brief Set the rotation direction of the display
 * @param m -> rotation parameter(please refer it in st7789.h)
 * @return none
 */
void ST7789_SetRotation(uint8_t m)
{
	ST7789_WriteCommand(ST7789_MADCTL);	// MADCTL
	switch (m) {
	case 0:
		ST7789_WriteSmallData(ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB);
		break;
	case 1:
		ST7789_WriteSmallData(ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
		break;
	case 2:
		ST7789_WriteSmallData(ST7789_MADCTL_RGB);
		break;
	case 3:
		ST7789_WriteSmallData(ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
		break;
	default:
		break;
	}
}

/**
 * @brief Set address of DisplayWindow
 * @param xi&yi -> coordinates of window
 * @return none
 */
static void ST7789_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	ST7789_Select();
	uint16_t x_start = x0 + X_SHIFT, x_end = x1 + X_SHIFT;
	uint16_t y_start = y0 + Y_SHIFT, y_end = y1 + Y_SHIFT;
	
	/* Column Address set */
	ST7789_WriteCommand(ST7789_CASET); 
	{
		uint8_t data[] = {x_start >> 8, x_start & 0xFF, x_end >> 8, x_end & 0xFF};
		ST7789_WriteData(data, sizeof(data));
	}

	/* Row Address set */
	ST7789_WriteCommand(ST7789_RASET);
	{
		uint8_t data[] = {y_start >> 8, y_start & 0xFF, y_end >> 8, y_end & 0xFF};
		ST7789_WriteData(data, sizeof(data));
	}
	/* Write to RAM */
	ST7789_WriteCommand(ST7789_RAMWR);
	ST7789_UnSelect();
}

/**
 * @brief Initialize ST7789 controller
 * @param none
 * @return none
 */
void ST7789_Init()
{
	HAL_Delay(25);
	ST7789_RST_Clr();
	HAL_Delay(25);
	ST7789_RST_Set();
	HAL_Delay(50);

	ST7789_WriteCommand(ST7789_COLMOD);		//	Set color mode
	ST7789_WriteSmallData(ST7789_COLOR_MODE_16bit);
	ST7789_WriteCommand(0xB2);				//	Porch control
	{
		uint8_t data[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
		ST7789_WriteData(data, sizeof(data));
	}
	ST7789_SetRotation(ST7789_ROTATION);	//	MADCTL (Display Rotation)

	/* Internal LCD Voltage generator settings */
	ST7789_WriteCommand(0XB7);				//	Gate Control
	ST7789_WriteSmallData(0x35);			//	Default value
	ST7789_WriteCommand(0xBB);				//	VCOM setting
	ST7789_WriteSmallData(0x19);			//	0.725v (default 0.75v for 0x20)
	ST7789_WriteCommand(0xC0);				//	LCMCTRL	
	ST7789_WriteSmallData (0x2C);			//	Default value
	ST7789_WriteCommand (0xC2);				//	VDV and VRH command Enable
	ST7789_WriteSmallData (0x01);			//	Default value
	ST7789_WriteCommand (0xC3);				//	VRH set
	ST7789_WriteSmallData (0x12);			//	+-4.45v (defalut +-4.1v for 0x0B)
	ST7789_WriteCommand (0xC4);				//	VDV set
	ST7789_WriteSmallData (0x20);			//	Default value
	ST7789_WriteCommand (0xC6);				//	Frame rate control in normal mode
	ST7789_WriteSmallData (0x0F);			//	Default value (60HZ)
	ST7789_WriteCommand (0xD0);				//	Power control
	ST7789_WriteSmallData (0xA4);			//	Default value
	ST7789_WriteSmallData (0xA1);			//	Default value
	/**************** Division line ****************/

	ST7789_WriteCommand(0xE0);
	{
		uint8_t data[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
		ST7789_WriteData(data, sizeof(data));
	}

	ST7789_WriteCommand(0xE1);
	{
		uint8_t data[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};
		ST7789_WriteData(data, sizeof(data));
	}
	ST7789_WriteCommand (ST7789_INVON);		//	Inversion ON
	ST7789_WriteCommand (ST7789_SLPOUT);	//	Out of sleep mode
	ST7789_WriteCommand (ST7789_NORON);		//	Normal Display on
	ST7789_WriteCommand (ST7789_DISPON);	//	Main screen turned on	

	HAL_Delay(50);
	ST7789_Fill_Color(0, 0, 240, 240, FLIP_RGB(BLACK));
}

/**
 * @brief Fill the DisplayWindow with single color
 * @param color -> color to Fill with
 * @return none
 */
void ST7789_Fill_Color(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
	uint16_t total_buff = w * h;
	ST7789_SetAddressWindow(x, y, x + w - 1, y + h - 1);
	ST7789_Select();

	#ifdef USE_DMA
		while(total_buff)
		{			
			if(total_buff > ST7789_WIDTH * HOR_LEN){
				memset(disp_buf, color, sizeof(disp_buf));
				total_buff -= ST7789_WIDTH * HOR_LEN;
				ST7789_WriteData((uint8_t *)disp_buf, sizeof(disp_buf));
			}
			else{
				memset(disp_buf, color, sizeof(uint16_t) * total_buff);
				ST7789_WriteData((uint8_t *)disp_buf, sizeof(uint16_t) * total_buff);
				total_buff = 0;
			}
		}
	#else
		uint16_t j;
		for (i = 0; i < ST7789_WIDTH; i++)
				for (j = 0; j < ST7789_HEIGHT; j++) {
					uint8_t data[] = {color >> 8, color & 0xFF};
					ST7789_WriteData(data, sizeof(data));
				}
	#endif
	ST7789_UnSelect();
}

/**
 * @brief Draw a Pixel
 * @param x&y -> coordinate to Draw
 * @param color -> color of the Pixel
 * @return none
 */
void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
	if ((x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT))	return;
	
	ST7789_SetAddressWindow(x, y, x, y);
	uint8_t data[] = {color >> 8, color & 0xFF};
	ST7789_WriteData(data, sizeof(data));
}

/**
 * @brief Draw a line with single color
 * @param x1&y1 -> coordinate of the start point
 * @param x2&y2 -> coordinate of the end point
 * @param color -> color of the line to Draw
 * @return none
 */
void ST7789_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color) 
{
	uint16_t swap;
	uint16_t steep = ABS(y1 - y0) > ABS(x1 - x0);
	if (steep) {
		swap = x0;
		x0 = y0;
		y0 = swap;

		swap = x1;
		x1 = y1;
		y1 = swap;
	}

	if (x0 > x1) {
		swap = x0;
		x0 = x1;
		x1 = swap;

		swap = y0;
		y0 = y1;
		y1 = swap;
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = ABS(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1)
		ystep = 1;
	else
		ystep = -1;
	

	for (; x0<=x1; x0++) 
	{
		if (steep) {
			ST7789_DrawPixel(y0, x0, color);
		} else {
			ST7789_DrawPixel(x0, y0, color);
		}
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
}




/**
 * @brief Draw an Image on the screen
 * @param x&y -> start point of the Image
 * @param w&h -> width & height of the Image to Draw
 * @param data -> pointer of the Image array
 * @return none
 */
void ST7789_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data)
{
	if ((x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT))
		return;
	if ((x + w - 1) >= ST7789_WIDTH)
		return;
	if ((y + h - 1) >= ST7789_HEIGHT)
		return;

	ST7789_Select();
	ST7789_SetAddressWindow(x, y, x + w - 1, y + h - 1);
	ST7789_WriteData((uint8_t *)data, sizeof(uint16_t) * w * h);
	ST7789_UnSelect();
}

void ST7789_WriteString(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, uint16_t bgcolor)
{
	uint16_t str_len = strlen(str);
	uint16_t* temp_arr=malloc(sizeof(uint16_t)* font.width * font.height * str_len);
	uint32_t i, b, j, ch_cnt;
	uint32_t color_buffer = color;
	uint32_t bgcolor_buffer = bgcolor;	
	
	for (i = 0; i < font.height; i++) {
		for(ch_cnt = 0; ch_cnt < str_len; ch_cnt++)
		{
			b = font.data[(str[ch_cnt] - 32) * font.height + i];
			for (j = 0; j < font.width; j++)
			{
				if ((b << j) & 0x8000) {
					temp_arr[i*font.width*str_len+ch_cnt*font.width+j]=color_buffer;
				}
				else {
					temp_arr[i*font.width*str_len+ch_cnt*font.width+j]=bgcolor_buffer;
				}
			}
		}
	}
	
	
	ST7789_Select();
	ST7789_SetAddressWindow(x, y, x + font.width * str_len - 1, y + font.height - 1);
	ST7789_WriteData((uint8_t *)temp_arr, sizeof(uint16_t) * font.width * font.height * str_len);
	ST7789_UnSelect();
	free(temp_arr);
	
}

