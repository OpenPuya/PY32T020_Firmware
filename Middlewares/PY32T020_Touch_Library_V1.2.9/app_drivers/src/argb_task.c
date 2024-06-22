#include "argb_task.h"
#if APP_SPI_LED_ENABLE
static uint8_t argb_start;
ARGBDisplay argb[SPI_LED_CNT];
uint16_t argb_time;
const LedSetData color_tbl[7] = {{255, 0, 0},   {0, 255, 0},   {0, 0, 255},    {255, 255, 0},
                                 {0, 255, 255}, {255, 0, 255}, {255, 255, 255}};

static void argb_color(LedSetData *rgb, uint8_t color)
{
    rgb->red = color_tbl[color].red;
    rgb->green = color_tbl[color].green;
    rgb->blue = color_tbl[color].blue;
}

void wheel_color(uint8_t count)
{
    uint8_t r, g, b, tmp;
    if (count > 23)
        return;
    if (count < 8)
    {
        r = 255;
        g = 0;
        b = 0;
        tmp = 32 * count;
        r -= tmp;
        g += tmp;
    }
    else if (count < 16)
    {
        r = 0;
        g = 255;
        b = 0;
        tmp = 32 * count;
        g -= tmp;
        b += tmp;
    }
    else
    {
        r = 0;
        g = 0;
        b = 255;
        tmp = 32 * count;
        b -= tmp;
        r += tmp;
    }
    tmp = count;
    if (tmp != 0)
    {
        tmp = 24 - tmp;
    }
    argb[tmp].color.red = r;
    argb[tmp].color.green = g;
    argb[tmp].color.blue = b;
}

void all_color(void)
{
    uint8_t i, j;
    for (i = 0; i < 24; i++)
    {
        wheel_color(i);
    }
    for (i = 0; i < 8; i++)
    {
        j = i * 3;
        argb[31 - i].color.red = argb[j].color.red;
        argb[31 - i].color.green = argb[j].color.green;
        argb[31 - i].color.blue = argb[j].color.blue;
    }
}

void argb_init(void)
{
    uint8_t i;
    for (i = 0; i < SPI_LED_CNT; i++)
    {
        argb[i].alpha = 0;
        argb[i].dim = 0;
        argb[i].flag_b.colorCount = 6;
    }
    all_color();
    argb_start = 0;
}

void argb_color_inc(void)
{
    uint8_t i;
    for (i = 0; i < SPI_LED_CNT; i++)
    {
        argb[i].flag_b.colorCount++;
        if (argb[i].flag_b.colorCount > 6)
            argb[i].flag_b.colorCount = 0;
        argb_color(&argb[i].color, argb[i].flag_b.colorCount);
        argb[i].flag_b.dir = 0;
        argb[i].alpha = 0;
    }
    if (argb[0].flag_b.colorCount == 6)
    {
        all_color();
    }
    argb_start = 0;
}

void argb_color_dec(void)
{
    uint8_t i;
    for (i = 0; i < SPI_LED_CNT; i++)
    {
        if (argb[i].flag_b.colorCount > 0)
            argb[i].flag_b.colorCount--;
        else
            argb[i].flag_b.colorCount = 6;
        argb_color(&argb[i].color, argb[i].flag_b.colorCount);
        argb[i].flag_b.dir = 0;
        argb[i].alpha = 0;
    }
    if (argb[0].flag_b.colorCount == 6)
    {
        all_color();
    }
    argb_start = 0;
}

void argb_config(uint8_t offset, uint8_t alpha, uint8_t dim, uint8_t dir, uint8_t hold)
{
    if (offset >= SPI_LED_CNT)
        return;
    argb[offset].alpha = alpha;
    argb[offset].hold = hold;
    argb[offset].flag_b.dir = dir;
    argb[offset].dim = dim;
    argb_start = SPI_LED_CNT;
}

void argb_serive(uint8_t whlelhold, uint8_t sliderhold)
{
    static uint8_t start_delay;
    uint8_t r, g, b;
    uint8_t i;
    if (argb_time < 9)
        return;
    argb_time = 0;
    for (i = 0; i < SPI_LED_CNT; i++)
    {
        if (whlelhold == i || sliderhold == i)
        {
        }
        else if (argb[i].hold)
        {
            argb[i].hold--;
        }
        else if (argb[i].flag_b.dir == 1)
        {
            if (argb[i].alpha > 5)
                argb[i].alpha -= 5;
            else
            {
                argb[i].flag_b.dir = 0;
                argb[i].alpha = 0;
            }
        }
        else if (argb[i].flag_b.dir == 2)
        {
            if ((argb[i].alpha + 5) < 255)
                argb[i].alpha += 5;
            else
            {
                argb[i].flag_b.dir = 1;
                argb[i].alpha = 255;
                argb[i].hold = 20;
            }
        }
        r = (argb[i].color.red * argb[i].alpha) / 255;
        g = (argb[i].color.green * argb[i].alpha) / 255;
        b = (argb[i].color.blue * argb[i].alpha) / 255;
        r = argb[i].dim * r / 255;
        g = argb[i].dim * g / 255;
        b = argb[i].dim * b / 255;
        SPI_LED_RgbLoad(i, r, g, b);
    }
    /*	¿ª»ú×Ô¼ì		*/
    if (argb_start < SPI_LED_CNT)
    {
        start_delay++;
        if (start_delay > 3)
        {
            start_delay = 0;
            argb[argb_start].alpha = 255;
            argb[argb_start].hold = 10;
            argb[argb_start].flag_b.dir = 1;
            argb[argb_start].dim = 128;
            argb_start++;
        }
    }
	SPI_LED_Transmit();
}
#endif
