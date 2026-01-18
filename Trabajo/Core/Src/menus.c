#include "menus.h"
#include "lcd.h"
#include "system_vars.h"
#include <string.h>
#include "posicion_pool.h"
#include "mapa.h"
#include "Radar.h"
#include "Laser.h"

typedef enum { MENU_1 = 0, MENU_2, MENU_3, MENU_4, MENU_5 } Menu;

static Menu menu_actual = MENU_1;

//previews (lo que ves mientras decides)
static uint8_t m2_preview = 0; // 0=MANUAL, 1=AUTO
static uint8_t m3_preview = 0; // 0=90, 1=180, 2=MANUAL

static const char *fire_str(FireMode m)
{
    return (m == FIRE_AUTO) ? "AUTO" : "MANUAL";
}

static const char *rot_str(RotMode r)
{
    switch (r) {
        case ROT_360:    return "360";
        case ROT_180:    return "180";
        default:         return "MANUAL";
    }
}

static const char *m2_preview_str(uint8_t pv)
{
    return (pv == 1) ? "AUTO" : "MANUAL";
}

static const char *m3_preview_str(uint8_t pv)
{
    if (pv == 0) return "360";
    if (pv == 1) return "180";
    return "MANUAL";
}

static void sync_previews_with_selected(void)
{
    // Al entrar a cada menú, el preview empieza en lo ya seleccionado (UX más lógica)
    m2_preview = (laser_get_estado() == FIRE_AUTO) ? 1u : 0u;
    m3_preview = (uint8_t)radar_get_estado(); // enum 0..2
}

static void Menus_Draw(void)
{
	uint32_t obj_totales=objetivo_objetivos_total();
	uint32_t abat_totales=objetivo_abatidos_total();
    switch (menu_actual)
    {
        case MENU_1:
            LCD_PrintfVar(0, "NUM OBJETIV: %3u", obj_totales);
            LCD_PrintfVar(1, "NUM ABATIDOS: %3u", abat_totales);
            break;

        case MENU_2:
            // Arriba: seleccionado real
            LCD_PrintfStr(0, "MODO DISP:%s", fire_str(laser_get_estado()));
            // Abajo: preview (cambia con corto)
            LCD_PrintfStr(1, "->%s 2s=OK", m2_preview_str(m2_preview));
            break;

        case MENU_3:
            // Arriba: seleccionado real
            LCD_PrintfStr(0, "ROTACION: %s", rot_str(radar_get_estado()));
            // Abajo: preview (cambia con corto)
            LCD_PrintfStr(1, "->%s 2s=OK", m3_preview_str(m3_preview));
            break;

        case MENU_4:
            // Menú 4: ángulo actual (usamos grados_rot)
            LCD_PrintfLine(0, "ANGULO ROTACION");
            LCD_PrintfVar (1, "GRADOS: %3u", (uint32_t)((grados_rot - 500.0) * 360.0 / 2000.0));
            break;

        case MENU_5:
            LCD_PrintfVar(0, "DIST MAX: %3u", (uint32_t)distancia_maxima);
            LCD_PrintfVar(0, "DIST ACTUAL: %3u", (uint32_t)distancia_actual);
            break;
    }
}

void Menus_Init(void)
{
    menu_actual = MENU_1;
    sync_previews_with_selected();

    LCD_Clear();
    Menus_Draw();
    LCD_Task();
}

void Menus_Task(TIM_HandleTypeDef *htim, BtnEvent evMenu, BtnEvent evSel, BtnEvent evRes, uint32_t now_ms)
{
    static uint32_t last_refresh_ms = 0;
    uint8_t redraw = 0;
    if (evRes == BTN_EVENT_SHORT){
    	menu_actual = MENU_1;
    	laser_set_estado(FIRE_MANUAL);
    	radar_set_estado(ROT_360);

    	mapa_reset();
    	laser_reset(htim);
    	pool_reset();
    	radar_reset(htim);
    }

    // PD0: cambiar menú con pulsación corta (ahora 4 menús)
    if (evMenu == BTN_EVENT_SHORT) {
        menu_actual = (Menu)((menu_actual + 1) % 4);

        // Al entrar en menu 2/3, sincroniza preview con lo seleccionado
        if (menu_actual == MENU_2 || menu_actual == MENU_3) {
            sync_previews_with_selected();
        }

        redraw = 1;
    }

    // PD1: según menú
    if (menu_actual == MENU_2)
    {
        if (evSel == BTN_EVENT_SHORT) {
            m2_preview = (uint8_t)((m2_preview + 1) % 2);
            redraw = 1;
        } else if (evSel == BTN_EVENT_LONG) {
        	laser_set_estado((m2_preview == 0) ? FIRE_MANUAL : FIRE_AUTO);
            redraw = 1;
        }
    }
    else if (menu_actual == MENU_3)
    {
        if (evSel == BTN_EVENT_SHORT) {
            m3_preview = (uint8_t)((m3_preview + 1) % 3);
            redraw = 1;
        } else if (evSel == BTN_EVENT_LONG) {
        	radar_set_estado((RotMode)m3_preview) ;
            redraw = 1;
        }
    }
    // MENU_4: no hace falta acción con SELECT (solo muestra)

    // Refresco automático para que menús 1 y 4 reflejen cambios sin pulsar
    if (!redraw && (now_ms - last_refresh_ms) >= 200u) {
        last_refresh_ms = now_ms;
        redraw = 1;
    }

    if (redraw) {
        Menus_Draw();
    }

    LCD_Task();
}
