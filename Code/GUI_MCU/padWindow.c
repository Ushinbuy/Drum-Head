#include "lvgl/examples/lv_examples.h"
#include "padWindow.h"
#include "guiObjects.h"
#include "stdio.h"
#include <stdlib.h>
#include <string.h>
#if LV_USE_MENU && LV_USE_MSGBOX && LV_BUILD_EXAMPLES

enum {
    LV_MENU_ITEM_BUILDER_VARIANT_1,
    LV_MENU_ITEM_BUILDER_VARIANT_2
};
typedef uint8_t lv_menu_builder_variant_t;

struct PadMemory_s {
	uint8_t sensitivity;   //0
	uint8_t threshold1;     //1
	uint8_t scantime;       //2
	uint8_t masktime;       //3
	uint8_t rimSensitivity; //4
	uint8_t rimThreshold;    //5
	uint8_t curvetype;       //6
	uint8_t note;           //7
	uint8_t noteRim;        //8
	uint8_t noteCup;        //9
	uint8_t soundHeadAddressId;	//10 this field show which item from soundsAdresses
	uint8_t soundRimAddressId;	//11
	uint8_t soundCupAddressId;	//12
	float soundHeadVolumeDb;
	float soundRimVolumeDb;
	float soundCupVolumeDb;
} PadMemory_default = {
		.sensitivity = 100,
		.threshold1 = 10,
		.scantime = 10,
		.masktime = 30,
		.rimSensitivity = 20,
		.rimThreshold = 3,
		.curvetype = 0,
		.note = 38,
		.noteRim = 39,
		.noteCup = 40,
		.soundHeadAddressId = 0xFF,
		.soundRimAddressId = 0xFF,
		.soundCupAddressId = 0xFF,
		.soundHeadVolumeDb = -3.,
		.soundRimVolumeDb = -3.,
		.soundCupVolumeDb = -3.
	};

typedef struct PadMemory_s PadMemory;

static void back_event_handler(lv_event_t * e);
static void switch_handler(lv_event_t * e);
lv_obj_t * root_page;
static lv_obj_t * create_text(lv_obj_t * parent, const char * icon, const char * txt,
                              lv_menu_builder_variant_t builder_variant);
static SliderWithText create_slider(lv_obj_t * parent,
                                const char * icon, const char * txt, int32_t min, int32_t max, int32_t val);
static lv_obj_t * create_switch(lv_obj_t * parent,
                                const char * icon, const char * txt, bool chk);

static void slider_event_cb(lv_event_t * e);

static void idButtonsEvent_cb(lv_event_t * e);

static IdButtonsObj create_inc_dec(lv_obj_t * parent, const char * txt);
static int32_t limit_value(int32_t v);

static SliderWithText sensitivity;   //0
static SliderWithText threshold;     //1
static SliderWithText scantime;       //2
static SliderWithText masktime;       //3
static SliderWithText rimSensitivity; //4
static SliderWithText rimThreshold;    //5
static IdButtonsObj curvetype;       //6
uint8_t note;           //7
uint8_t noteRim;        //8
uint8_t noteCup;        //9
static IdButtonsObj soundHeadAddressId;	//10 this field show which item from soundsAdresses
static IdButtonsObj soundRimAddressId;	//11
static IdButtonsObj soundCupAddressId;	//12
float soundHeadVolumeDb;
float soundRimVolumeDb;
float soundCupVolumeDb;



static SliderWithText brigthness;

#define NUM_SLIDERS 7
#define NUM_ID_BUTTONS 1
static SliderWithText * slidersList[NUM_SLIDERS];
static IdButtonsObj * idButtonsList[NUM_ID_BUTTONS];

static void initPadWindow(void){
	slidersList[0] = &sensitivity;
	slidersList[1] = &threshold;
	slidersList[2] = &scantime;
	slidersList[3] = &masktime;
	slidersList[4] = &rimSensitivity;
	slidersList[5] = &rimThreshold;
	slidersList[6] = &brigthness;
}

static void initIdButtonsList(void){
	idButtonsList[0] = &curvetype;
//	idButtonsList[1] = soundRimAddressId;
//	idButtonsList[2] = soundCupAddressId;
}

void lv_example_menu_7(void)
{
	PadMemory newPad = PadMemory_default;
	printf("pad value is %d", newPad.note);

    lv_obj_t * menu = lv_menu_create(lv_scr_act());

    lv_color_t bg_color = lv_obj_get_style_bg_color(menu, 0);
    if(lv_color_brightness(bg_color) > 127) {
        lv_obj_set_style_bg_color(menu, lv_color_darken(lv_obj_get_style_bg_color(menu, 0), 10), 0);
    }
    else {
        lv_obj_set_style_bg_color(menu, lv_color_darken(lv_obj_get_style_bg_color(menu, 0), 50), 0);
    }
    lv_menu_set_mode_root_back_btn(menu, LV_MENU_ROOT_BACK_BTN_ENABLED);
    lv_obj_add_event(menu, back_event_handler, LV_EVENT_CLICKED, menu);
    lv_obj_set_size(menu, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
    lv_obj_center(menu);

    lv_obj_t * cont;
    lv_obj_t * section;

    /*Create sub pages*/
    lv_obj_t * sub_mechanics_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_mechanics_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_separator_create(sub_mechanics_page);
    section = lv_menu_section_create(sub_mechanics_page);

    initPadWindow();
    initIdButtonsList();
    sensitivity = create_slider(section, NULL, "Sensitivity", 0, 150, 120);
    threshold = create_slider(section, NULL, "Threshold", 0, 150, 50);
    scantime = create_slider(section, NULL, "Scan time", 0, 150, 80);
    masktime = create_slider(section, NULL, "Mask time", 0, 150, 80);
    rimSensitivity = create_slider(section, NULL, "Rim Sensitivity", 0, 150, 80);
    rimThreshold = create_slider(section, NULL, "Rim Threshold", 0, 150, 80);
    curvetype = create_inc_dec(section, "Curve Type");

    lv_obj_t * sub_sound_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_sound_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_separator_create(sub_sound_page);
    section = lv_menu_section_create(sub_sound_page);
    create_switch(section, LV_SYMBOL_AUDIO, "Sound", false);

    lv_obj_t * sub_display_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_display_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_separator_create(sub_display_page);
    section = lv_menu_section_create(sub_display_page);
    brigthness = create_slider(section, NULL, "Brightness", 0, 150, 100);

    lv_obj_t * sub_software_info_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_software_info_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    section = lv_menu_section_create(sub_software_info_page);
    create_text(section, NULL, "Version 1.0", LV_MENU_ITEM_BUILDER_VARIANT_1);

    lv_obj_t * sub_legal_info_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_legal_info_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    section = lv_menu_section_create(sub_legal_info_page);
    for(uint32_t i = 0; i < 15; i++) {
        create_text(section, NULL,
                    "This is a long long long long long long long long long text, if it is long enough it may scroll.",
                    LV_MENU_ITEM_BUILDER_VARIANT_1);
    }

    lv_obj_t * sub_about_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_about_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_separator_create(sub_about_page);
    section = lv_menu_section_create(sub_about_page);
    cont = create_text(section, NULL, "Software information", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_software_info_page);
    cont = create_text(section, NULL, "Legal information", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_legal_info_page);

    lv_obj_t * sub_menu_mode_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_menu_mode_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_separator_create(sub_menu_mode_page);
    section = lv_menu_section_create(sub_menu_mode_page);
    cont = create_switch(section, LV_SYMBOL_AUDIO, "Sidebar enable", true);
    lv_obj_add_event(lv_obj_get_child(cont, 2), switch_handler, LV_EVENT_VALUE_CHANGED, menu);

    /*Create a root page*/
    root_page = lv_menu_page_create(menu, "Settings");
    lv_obj_set_style_pad_hor(root_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    section = lv_menu_section_create(root_page);
    cont = create_text(section, LV_SYMBOL_SETTINGS, "Sensitivity", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_mechanics_page);
    cont = create_text(section, LV_SYMBOL_AUDIO, "Sound", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_sound_page);
    cont = create_text(section, LV_SYMBOL_SETTINGS, "Display", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_display_page);

    create_text(root_page, NULL, "Others", LV_MENU_ITEM_BUILDER_VARIANT_1);
    section = lv_menu_section_create(root_page);
    cont = create_text(section, NULL, "About", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_about_page);
    cont = create_text(section, LV_SYMBOL_SETTINGS, "Menu mode", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_menu_mode_page);

    lv_menu_set_sidebar_page(menu, root_page);

    lv_obj_send_event(lv_obj_get_child(lv_obj_get_child(lv_menu_get_cur_sidebar_page(menu), 0), 0), LV_EVENT_CLICKED,
                      NULL);
}

static void slider_event_cb(lv_event_t * e)
{
    lv_obj_t * slider = lv_event_get_target(e);

    uint32_t newVal = lv_slider_get_value(slider);

    char buff[4];
    sprintf(buff, "%d", newVal);

    for(uint8_t sliderNum = 0; sliderNum < NUM_SLIDERS; sliderNum++){
		if (slider->parent == slidersList[sliderNum]->sliderObj) {
			lv_label_set_text(slidersList[sliderNum]->sliderValueText, buff);
			break;
		}
    }
}

static void back_event_handler(lv_event_t * e)
{
    lv_obj_t * obj = lv_event_get_target(e);
    lv_obj_t * menu = lv_event_get_user_data(e);

    if(lv_menu_back_btn_is_root(menu, obj)) {
        lv_obj_t * mbox1 = lv_msgbox_create(NULL, "Hello", "Root back btn click.", NULL, true);
        lv_obj_center(mbox1);
    }
}

static void switch_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * menu = lv_event_get_user_data(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        if(lv_obj_has_state(obj, LV_STATE_CHECKED)) {
            lv_menu_set_page(menu, NULL);
            lv_menu_set_sidebar_page(menu, root_page);
            lv_obj_send_event(lv_obj_get_child(lv_obj_get_child(lv_menu_get_cur_sidebar_page(menu), 0), 0), LV_EVENT_CLICKED,
                              NULL);
        }
        else {
            lv_menu_set_sidebar_page(menu, NULL);
            lv_menu_clear_history(menu); /* Clear history because we will be showing the root page later */
            lv_menu_set_page(menu, root_page);
        }
    }
}

static lv_obj_t * create_text(lv_obj_t * parent, const char * icon, const char * txt,
                              lv_menu_builder_variant_t builder_variant)
{
    lv_obj_t * obj = lv_menu_cont_create(parent);

    lv_obj_t * img = NULL;
    lv_obj_t * label = NULL;

    if(icon) {
        img = lv_img_create(obj);
        lv_img_set_src(img, icon);
    }

    if(txt) {
        label = lv_label_create(obj);
        lv_label_set_text(label, txt);
        lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_set_flex_grow(label, 0);
    }

    if(builder_variant == LV_MENU_ITEM_BUILDER_VARIANT_2 && icon && txt) {
        lv_obj_add_flag(img, LV_OBJ_FLAG_FLEX_IN_NEW_TRACK);
        lv_obj_swap(img, label);
    }

    return obj;
}

static IdButtonsObj create_inc_dec(lv_obj_t * parent, const char * txt){
	lv_obj_t * obj = create_text(parent, NULL, txt, LV_MENU_ITEM_BUILDER_VARIANT_2);

	lv_obj_t * btnPlus;
	lv_obj_t * btnPlusString;
	lv_obj_t * btnMinus;
	lv_obj_t * btnMinusString;
	lv_obj_t * label;

	/*Up button*/
	btnMinus = lv_btn_create(obj);
	lv_obj_set_flex_grow(btnMinus, 1);
	lv_obj_add_event(btnMinus, idButtonsEvent_cb, LV_EVENT_ALL, NULL);
	btnPlusString = lv_label_create(btnMinus);
	lv_label_set_text(btnPlusString, LV_SYMBOL_LEFT);
	lv_obj_set_height(btnMinus, 20);
	lv_obj_center(btnPlusString);

	label = lv_label_create(obj);
	lv_obj_set_flex_grow(label, 2);
	lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
	uint8_t startValue = 35;
	char startValueString[3];
	sprintf(startValueString, "%d", startValue);
	lv_label_set_text(label, startValueString);
//	lv_msg_subscribe_obj((lv_msg_id_t)&power_value, label, NULL);
//	lv_obj_add_event(label, label_event_cb, LV_EVENT_MSG_RECEIVED, NULL);

	btnPlus = lv_btn_create(obj);
	lv_obj_set_flex_grow(btnPlus, 1);
	lv_obj_add_event(btnPlus, idButtonsEvent_cb, LV_EVENT_ALL, NULL);
	btnMinusString = lv_label_create(btnPlus);
	lv_label_set_text(btnMinusString, LV_SYMBOL_RIGHT);
	lv_obj_set_height(btnPlus, 20);
	lv_obj_center(btnMinusString);

//	lv_msg_update_value(&power_value);

	IdButtonsObj returnObject;
	returnObject.valueText = label;
	returnObject.incButton = btnPlus;
	returnObject.decButton = btnMinus;

	return returnObject;
}

static int32_t limit_value(int32_t v)
{
    return LV_CLAMP(30, v, 80);
}

static void idButtonsEvent_cb(lv_event_t * e)
{
    lv_obj_t * btn = lv_event_get_target(e);
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    	for(uint8_t idBtn_num = 0; idBtn_num < NUM_ID_BUTTONS; idBtn_num++){
//    		printf("\n current button is %X and parent is %X", btn, btn->parent);
//    		printf("\n check   button is %X and %X", idButtonsList[idBtn_num]->decButton, idButtonsList[idBtn_num]->incButton);
			if (btn == idButtonsList[idBtn_num]->decButton) { /*First object is the dec. button*/
				char buff[3];
				strcpy(buff, lv_label_get_text(idButtonsList[idBtn_num]->valueText));
				uint8_t val = atoi(buff);

				val = limit_value(--val);
				sprintf(buff, "%d", val);
				lv_label_set_text(idButtonsList[idBtn_num]->valueText, buff);
			}
			else if (btn == idButtonsList[idBtn_num]->incButton) {
				char buff[3];
				strcpy(buff,
						lv_label_get_text(idButtonsList[idBtn_num]->valueText));
				uint8_t val = atoi(buff);

				val = limit_value(++val);
				sprintf(buff, "%d", val);
				lv_label_set_text(idButtonsList[idBtn_num]->valueText, buff);
			}
    	}
    }
}

static SliderWithText create_slider(lv_obj_t * parent, const char * icon, const char * txt, int32_t min, int32_t max,
                                int32_t val)
{
    lv_obj_t * obj = create_text(parent, icon, txt, LV_MENU_ITEM_BUILDER_VARIANT_2);

    lv_obj_t * labelVal = lv_label_create(obj);
	char buffer[4];
	sprintf(buffer, "%d", val);
	lv_label_set_text(labelVal, buffer);
	lv_label_set_long_mode(labelVal, LV_LABEL_LONG_SCROLL_CIRCULAR);
	lv_obj_set_flex_grow(labelVal, 0);

    lv_obj_t * slider = lv_slider_create(obj);
    lv_obj_set_flex_grow(slider, 10);
    lv_slider_set_range(slider, min, max);
    lv_slider_set_value(slider, val, LV_ANIM_OFF);

    lv_obj_set_width(slider, 20);

    lv_obj_add_event(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);


    if(icon == NULL) {
        lv_obj_add_flag(slider, LV_OBJ_FLAG_FLEX_IN_NEW_TRACK);
    }

    SliderWithText returnSlider;
    returnSlider.sliderObj = obj;
    returnSlider.sliderValueText = labelVal;
    return returnSlider;
}

static lv_obj_t * create_switch(lv_obj_t * parent, const char * icon, const char * txt, bool chk)
{
    lv_obj_t * obj = create_text(parent, icon, txt, LV_MENU_ITEM_BUILDER_VARIANT_1);

    lv_obj_t * sw = lv_switch_create(obj);
    lv_obj_add_state(sw, chk ? LV_STATE_CHECKED : 0);

    return obj;
}

#endif
