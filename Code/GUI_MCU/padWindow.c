#include "lvgl/examples/lv_examples.h"
#include "padWindow.h"
#include "guiObjects.h"
#include "stdio.h"
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

static void btn_event_cb(lv_event_t * e);
static void label_event_cb(lv_event_t * e);
static lv_obj_t * create_inc_dec(lv_obj_t * parent, const char * txt);
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

static SliderWithText slidersList[];

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
    printf("OOOOP");
    sensitivity = create_slider(section, NULL, "Sensitivity", 0, 150, 120);
    threshold = create_slider(section, NULL, "Threshold", 0, 150, 50);
    scantime = create_slider(section, NULL, "Scan time", 0, 150, 80);
    create_inc_dec(section, "Hello");

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
    cont = create_text(section, LV_SYMBOL_SETTINGS, "Mechanics", LV_MENU_ITEM_BUILDER_VARIANT_1);
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

    if(slider->parent == sensitivity.sliderObj){
    	lv_label_set_text(sensitivity.sliderValueText, buff);
    }
    else if(slider->parent == threshold.sliderObj){
    	lv_label_set_text(threshold.sliderValueText, buff);
    }
    else if (slider->parent == scantime.sliderObj) {
		lv_label_set_text(scantime.sliderValueText, buff);
	}
    else if (slider->parent == brigthness.sliderObj) {
		lv_label_set_text(brigthness.sliderValueText, buff);
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

static int32_t power_value; // TODO

static lv_obj_t * create_inc_dec(lv_obj_t * parent, const char * txt){
	lv_obj_t * obj = create_text(parent, NULL, txt, LV_MENU_ITEM_BUILDER_VARIANT_2);

	lv_obj_t * btn;
	lv_obj_t * label;

	/*Up button*/
	btn = lv_btn_create(obj);
	lv_obj_set_flex_grow(btn, 1);
	lv_obj_add_event(btn, btn_event_cb, LV_EVENT_ALL, NULL);
	label = lv_label_create(btn);
	lv_label_set_text(label, LV_SYMBOL_LEFT);
	lv_obj_set_height(btn, 20);
	lv_obj_center(label);

	label = lv_label_create(obj);
	lv_obj_set_flex_grow(label, 2);
	lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
	lv_label_set_text(label, "?");
	lv_msg_subscribe_obj((lv_msg_id_t)&power_value, label, NULL);
	lv_obj_add_event(label, label_event_cb, LV_EVENT_MSG_RECEIVED, NULL);

	btn = lv_btn_create(obj);
	lv_obj_set_flex_grow(btn, 1);
	lv_obj_add_event(btn, btn_event_cb, LV_EVENT_ALL, NULL);
	label = lv_label_create(btn);
	lv_label_set_text(label, LV_SYMBOL_RIGHT);
	lv_obj_set_height(btn, 20);
	lv_obj_center(label);

	power_value = 30;
	lv_msg_update_value(&power_value);

	return obj;
}

static int32_t limit_value(int32_t v)
{
    return LV_CLAMP(30, v, 80);
}

static void btn_event_cb(lv_event_t * e)
{
    lv_obj_t * btn = lv_event_get_target(e);
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    	printf("\n call button is %X", btn);
    	printf("\n index button is %d", lv_obj_get_index(btn));
        if(lv_obj_get_index(btn) == 0) {    /*First object is the dec. button*/
            power_value = limit_value(power_value - 1);
            lv_msg_update_value(&power_value);
        }
        else {
            power_value = limit_value(power_value + 1);
            lv_msg_update_value(&power_value);
        }
    }
}

static void label_event_cb(lv_event_t * e)
{
    lv_obj_t * label = lv_event_get_target(e);
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_MSG_RECEIVED) {
        lv_msg_t * m = lv_event_get_msg(e);
        const int32_t * v = lv_msg_get_payload(m);
        lv_label_set_text_fmt(label, "%"LV_PRId32" %%", *v);
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
