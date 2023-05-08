#include "mainWindow.h"
#include "lvgl/examples/lv_examples.h"
#include "padWindow.h"
#include <stdlib.h>
#include "padVirtual.h"

#if LV_USE_LIST && LV_BUILD_EXAMPLES

static lv_obj_t * currentButton = NULL;
static lv_obj_t * padScreen;
static lv_obj_t * mainScreen;
static uint8_t _numberOfPads;

static void back_from_pad_to_main_screen(){
	load_main_window(_numberOfPads);
	lv_obj_del(padScreen);
}

PadMemory pad;
uint8_t padWasInit = 0;

static void event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        LV_LOG_USER("Clicked: %s", lv_list_get_btn_text(list1, obj));

        if(currentButton == obj) {
            currentButton = NULL;
        }
        else {
            currentButton = obj;
        }
        lv_obj_t * parent = lv_obj_get_parent(obj);
        uint32_t i;
        for(i = 0; i < lv_obj_get_child_cnt(parent); i++) {
            lv_obj_t * child = lv_obj_get_child(parent, i);
            if(child == currentButton) {
            	padScreen = lv_obj_create(NULL);
				load_pad_screen(padScreen, &pad, back_from_pad_to_main_screen);

				lv_scr_load(padScreen);
				lv_obj_del(mainScreen);
            }
        }
    }
}

static void initPad(){
	pad.sensitivity = 100;
	pad.threshold1 = 10;
	pad.scantime = 10;
	pad.masktime = 30;
	pad.rimSensitivity = 20;
	pad.rimThreshold = 3;
	pad.curvetype = 1;
	pad.note = 38;
	pad.noteRim = 39;
	pad.noteCup = 40;
	pad.soundHeadAddressId = 0x2;
	pad.soundRimAddressId = 0x3;
	pad.soundCupAddressId = 0xFF;
	pad.soundHeadVolumeDb = -3.;
	pad.soundRimVolumeDb = -3.;
	pad.soundCupVolumeDb = -3.;

	padWasInit = 1;
}

void load_main_window(uint8_t numberOfPads)
{
    /*Create a list*/
	_numberOfPads = numberOfPads;
	if(padWasInit == 0){
		initPad();
	}

    mainScreen = lv_list_create(NULL);
    lv_obj_set_size(mainScreen, lv_pct(60), lv_pct(100));
    lv_obj_set_style_pad_row(mainScreen, 5, 0);

    /*Add buttons to the list*/
    lv_obj_t * btn;
    int i;
    for(i = 0; i < numberOfPads; i++) {
        btn = lv_btn_create(mainScreen);
        lv_obj_set_width(btn, lv_pct(30));
        lv_obj_set_align(btn, LV_ALIGN_CENTER);
        lv_obj_add_event(btn, event_handler, LV_EVENT_CLICKED, NULL);

        lv_obj_t * lab = lv_label_create(btn);
        lv_label_set_text_fmt(lab, "Pad %d", i);
    }

    lv_scr_load(mainScreen);
}

#endif
