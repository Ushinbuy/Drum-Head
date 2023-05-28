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

static lv_obj_t * root_page;

PadState * _padState;

static SliderWithText sensitivity;   //0
static SliderWithText threshold;     //1
static SliderWithText scantime;       //2
static SliderWithText masktime;       //3
static SliderWithText rimSensitivity; //4
static SliderWithText rimThreshold;    //5
static IdButtonsObj curvetype;       //6
static SliderWithText note;           //7
static SliderWithText noteRim;        //8
static SliderWithText noteCup;        //9
static IdButtonsObj soundHeadAddressId;	//10 this field show which item from soundsAdresses
static IdButtonsObj soundRimAddressId;	//11
static IdButtonsObj soundCupAddressId;	//12
static SliderWithText soundHeadVolumeDb;
static SliderWithText soundRimVolumeDb;
static SliderWithText soundCupVolumeDb;

static const uint8_t multiplier_float_int = 10;


static SliderWithText brigthness;

#define NUM_SLIDERS 10
#define NUM_SLIDERS_FLOAT 3
#define NUM_ID_BUTTONS 4
static SliderWithText * slidersList[NUM_SLIDERS];
static SliderWithText * slidersListFloat[NUM_SLIDERS_FLOAT];
static IdButtonsObj * idButtonsList[NUM_ID_BUTTONS];

static void (*_backToMain)();
lv_obj_t * saveBtn;
lv_obj_t * cancelBtn;

static void showSaveAndCancelButtons(void){
	if(*_padState == PAD_NOT_CHANGED){
		*_padState = PAD_WAS_CHANGED;
		lv_obj_clear_flag(saveBtn, LV_OBJ_FLAG_HIDDEN);
		lv_obj_clear_flag(cancelBtn, LV_OBJ_FLAG_HIDDEN);
	}
}

static void slider_event_cb(lv_event_t * e)
{
    lv_obj_t * slider = lv_event_get_target(e);

    uint32_t newVal = lv_slider_get_value(slider);

    char buff[4];
    sprintf(buff, "%d", newVal);

    for(uint8_t sliderNum = 0; sliderNum < NUM_SLIDERS; sliderNum++){
    	if(slidersList[sliderNum] == NULL){
    		continue;
    	}
    	else if (slider->parent == slidersList[sliderNum]->sliderObj) {
			lv_label_set_text(slidersList[sliderNum]->sliderValueText, buff);
			*(uint8_t *) slidersList[sliderNum]->addressPadParameter = newVal;
			showSaveAndCancelButtons();
//			printUpdatedPad();
			break;
		}
    }
}

static void slider_event_float_cb(lv_event_t * e)
{
    lv_obj_t * slider = lv_event_get_target(e);

    int32_t newVal = lv_slider_get_value(slider);
    float newValFloat = (float)(newVal) / multiplier_float_int;

    char newValStr[20];
    sprintf(newValStr, "%0.1f", newValFloat);

    for(uint8_t sliderNum = 0; sliderNum < NUM_SLIDERS_FLOAT; sliderNum++){
    	if(slidersListFloat[sliderNum] == NULL){
    		continue;
    	}
    	else if (slider->parent == slidersListFloat[sliderNum]->sliderObj) {
			lv_label_set_text(slidersListFloat[sliderNum]->sliderValueText, newValStr);
			*(float *) slidersListFloat[sliderNum]->addressPadParameter = newValFloat;
			showSaveAndCancelButtons();
//			printUpdatedPad();
			break;
		}
    }
}

static void idButtonsEvent_cb(lv_event_t * e)
{
    lv_obj_t * btn = lv_event_get_target(e);
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    	for(uint8_t idBtn_num = 0; idBtn_num < NUM_ID_BUTTONS; idBtn_num++){
    		if(idButtonsList[idBtn_num] == NULL){
    			continue;
    		}
    		else if (btn == idButtonsList[idBtn_num]->decButton) { /*First object is the dec. button*/
				char buff[4];
				strcpy(buff, lv_label_get_text(idButtonsList[idBtn_num]->valueText));
				uint8_t val = atoi(buff);

				if(val == 0){
					val = idButtonsList[idBtn_num]->maxValue;
				}
				else{
					val--;
				}
				sprintf(buff, "%d", val);
				lv_label_set_text(idButtonsList[idBtn_num]->valueText, buff);
				*(uint8_t *) idButtonsList[idBtn_num]->addressPadParameter = val;
				showSaveAndCancelButtons();
//				printUpdatedPad();
			}
			else if (btn == idButtonsList[idBtn_num]->incButton) {
				char buff[4];
				strcpy(buff, lv_label_get_text(idButtonsList[idBtn_num]->valueText));
				uint8_t val = atoi(buff);

				if(val == idButtonsList[idBtn_num]->maxValue){
					val = 0;
				}
				else{
					val++;
				}
				sprintf(buff, "%d", val);
				lv_label_set_text(idButtonsList[idBtn_num]->valueText, buff);
				*(uint8_t*) idButtonsList[idBtn_num]->addressPadParameter = val;
				showSaveAndCancelButtons();
//				printUpdatedPad();
			}
    	}
    }
}

static void create_keyboard(lv_obj_t * obj){
    static const char * kb_map[] = {
                  "1","2", "3","\n",
                  "4", "5", "6","\n",
                  "7", "8", "9","\n",
				  LV_SYMBOL_CLOSE ,"0", LV_SYMBOL_OK, NULL
        };
    static const lv_btnmatrix_ctrl_t kb_ctrl[] = {4, 4, 4,
                                                  4, 4, 4,
                                                  4, 4, 4,
                                                  4, 4, 4
                                                 };

    lv_obj_t * keyboard = lv_keyboard_create(obj);

    lv_keyboard_set_map(keyboard, LV_KEYBOARD_MODE_USER_1, kb_map, kb_ctrl);
	lv_keyboard_set_mode(keyboard, LV_KEYBOARD_MODE_USER_1);


    lv_obj_set_size(keyboard,100,100);

//    lv_obj_t * ta;
//    ta = lv_textarea_create(scr);
//    lv_obj_align(ta, LV_ALIGN_TOP_MID, 0, 10);
//    lv_obj_set_size(ta, lv_pct(90), 80);
//    lv_obj_add_state(ta, LV_STATE_FOCUSED);
//
//    lv_keyboard_set_textarea(keyboard, ta);
}

static void back_event_handler(lv_event_t * e)
{
//    lv_obj_t * obj = lv_event_get_target(e);
//    lv_obj_t * menu = lv_event_get_user_data(e);
//
//    if(lv_menu_back_btn_is_root(menu, obj)) {
//        lv_obj_t * mbox1 = lv_msgbox_create(NULL, "Hello", "Root back btn click.", NULL, true);
//        lv_obj_center(mbox1);
//    }
	(*_backToMain)();
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

static IdButtonsObj create_inc_dec(lv_obj_t * parent, const char * txt, uint8_t maxValue, void * addressParameter){
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
	uint8_t val = *(uint8_t *) addressParameter;
	char startValueString[4];
	sprintf(startValueString, "%d", val);
	lv_label_set_text(label, startValueString);

	btnPlus = lv_btn_create(obj);
	lv_obj_set_flex_grow(btnPlus, 1);
	lv_obj_add_event(btnPlus, idButtonsEvent_cb, LV_EVENT_ALL, NULL);
	btnMinusString = lv_label_create(btnPlus);
	lv_label_set_text(btnMinusString, LV_SYMBOL_RIGHT);
	lv_obj_set_height(btnPlus, 20);
	lv_obj_center(btnMinusString);

	IdButtonsObj returnObject;
	returnObject.valueText = label;
	returnObject.incButton = btnPlus;
	returnObject.decButton = btnMinus;
	returnObject.maxValue = maxValue;
	returnObject.addressPadParameter = addressParameter;

	return returnObject;
}

static lv_obj_t * create_text_edit(lv_obj_t * parent, const char * txt, void * addressParameter){
	lv_obj_t * text = create_text(parent, NULL, txt, LV_MENU_ITEM_BUILDER_VARIANT_2);

	/*Up button*/
	lv_obj_t * textArea = lv_textarea_create(text);

//	static lv_style_t style;
//	lv_style_init(&style);
//	lv_style_set_text_font(&style, &lv_font_montserrat_10);
//	lv_obj_add_style(textArea, &style, LV_PART_SELECTED);

	lv_obj_set_size(textArea, 70, 40);

	uint8_t value = *(uint8_t*) addressParameter;

	char startValueString[4];
	sprintf(startValueString, "%d", value);

	lv_textarea_add_text(textArea, startValueString);

//	IdButtonsObj returnObject;
//	returnObject.valueText = label;

//	returnObject.addressPadParameter = addressParameter;

	return textArea;
}

static SliderWithText create_slider(lv_obj_t *parent, const char *icon,
		const char *txt, int32_t min, int32_t max, void *addressParameter)
{
    lv_obj_t * obj = create_text(parent, icon, txt, LV_MENU_ITEM_BUILDER_VARIANT_2);

    lv_obj_t * labelVal = lv_label_create(obj);

    uint8_t val = *(uint8_t *) addressParameter;
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
    returnSlider.addressPadParameter = addressParameter;
    return returnSlider;
}

static SliderWithText create_slider_float(lv_obj_t * parent, const char * icon, const char * txt, float min, float max,
                                float val, void * addressParameter)
{
    lv_obj_t * obj = create_text(parent, icon, txt, LV_MENU_ITEM_BUILDER_VARIANT_2);

    lv_obj_t * labelVal = lv_label_create(obj);
	char buffer[20];

	sprintf(buffer, "%0.1f", val);
	lv_label_set_text(labelVal, buffer);
	lv_label_set_long_mode(labelVal, LV_LABEL_LONG_SCROLL_CIRCULAR);
	lv_obj_set_flex_grow(labelVal, 0);

    lv_obj_t * slider = lv_slider_create(obj);
    lv_obj_set_flex_grow(slider, 10);

    if(min < -100.1){
    	printf("minimum is incorrect value, must be more than -100.0");
    }
    else if(max > 10.1){
    	printf("maximum is incorrect value, must be less than 10.0");
    }

    int16_t min_int = multiplier_float_int * (int16_t) min;
    int16_t max_int = multiplier_float_int * (int16_t) max;
    int16_t val_int = multiplier_float_int * (int16_t) val;
    lv_slider_set_range(slider, min_int, max_int);
    lv_slider_set_value(slider, val_int, LV_ANIM_OFF);

    lv_obj_set_width(slider, 20);

    lv_obj_add_event(slider, slider_event_float_cb, LV_EVENT_VALUE_CHANGED, NULL);


    if(icon == NULL) {
        lv_obj_add_flag(slider, LV_OBJ_FLAG_FLEX_IN_NEW_TRACK);
    }

    SliderWithText returnSlider;
    returnSlider.sliderObj = obj;
    returnSlider.sliderValueText = labelVal;
    returnSlider.addressPadParameter = addressParameter;
    return returnSlider;
}

static void initPadWindow(void){
	slidersList[0] = &sensitivity;
	slidersList[1] = &threshold;
	slidersList[2] = &scantime;
	slidersList[3] = &masktime;
	slidersList[4] = &rimSensitivity;
	slidersList[5] = &rimThreshold;
	slidersList[6] = &brigthness;
	slidersList[7] = &note;
	slidersList[8] = &noteRim;
	slidersList[9] = &noteCup;

	slidersListFloat[0] = &soundHeadVolumeDb;
	slidersListFloat[1] = &soundRimVolumeDb;
	slidersListFloat[2] = &soundCupVolumeDb;

	idButtonsList[0] = &curvetype;
	idButtonsList[1] = &soundHeadAddressId;
	idButtonsList[2] = &soundRimAddressId;
	idButtonsList[3] = &soundCupAddressId;
}

static void save_btn_cb(lv_event_t * e){
	lv_event_code_t code = lv_event_get_code(e);
	if (code == LV_EVENT_CLICKED) {
		printf("\n was saved");	// TODO here must be callback

		lv_obj_add_flag(saveBtn, LV_OBJ_FLAG_HIDDEN);
		lv_obj_add_flag(cancelBtn, LV_OBJ_FLAG_HIDDEN);
		*_padState = PAD_NOT_CHANGED;
	}
}

static void cancel_btn_cb(lv_event_t * e){
	lv_event_code_t code = lv_event_get_code(e);
	if (code == LV_EVENT_CLICKED) {
		printf("\n was canceled");	// TODO here must be callback

		lv_obj_add_flag(saveBtn, LV_OBJ_FLAG_HIDDEN);
		lv_obj_add_flag(cancelBtn, LV_OBJ_FLAG_HIDDEN);
		*_padState = PAD_NOT_CHANGED;
	}
}

void load_pad_screen(lv_obj_t * scr, PadMemory * currentPad, PadState * padState, void (*backToMain)())
{
	_backToMain = backToMain;
	_padState = padState;

	if(scr == NULL){
		scr = lv_scr_act();
		// todo this check doesn't work
	}
    lv_obj_t * menu = lv_menu_create(scr);

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

    sensitivity = create_slider(section, NULL, "Sensitivity", 0, 150, &currentPad->sensitivity);
    threshold = create_slider(section, NULL, "Threshold", 0, 50, &currentPad->threshold1);
    scantime = create_slider(section, NULL, "Scan time", 0, 50, &currentPad->scantime);
    masktime = create_slider(section, NULL, "Mask time", 0, 150, &currentPad->masktime);
    rimSensitivity = create_slider(section, NULL, "Rim Sensitivity", 0, 150, &currentPad->rimSensitivity);
    rimThreshold = create_slider(section, NULL, "Rim Threshold", 0, 150, &currentPad->rimThreshold);
    curvetype = create_inc_dec(section, "Curve Type", 4, &currentPad->curvetype);

    lv_obj_t * sub_note_page = lv_menu_page_create(menu, NULL);
	lv_obj_set_style_pad_hor(sub_note_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
	lv_menu_separator_create(sub_note_page);
	section = lv_menu_section_create(sub_note_page);

	// TODO add later notes description like C#1
	note = create_slider(section, NULL, "Note MIDI Head", 21, 127, &currentPad->note);
	noteRim = create_slider(section, NULL, "Note MIDI Rim", 21, 127, &currentPad->noteRim);
	noteCup = create_slider(section, NULL, "Note MIDI Cup", 21, 127, &currentPad->noteCup);

//	create_keyboard(section);

    lv_obj_t * sub_sound_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_sound_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_separator_create(sub_sound_page);
    section = lv_menu_section_create(sub_sound_page);

    soundHeadAddressId = create_inc_dec(section, "Head sound ID", 5, &currentPad->soundHeadAddressId);
    soundRimAddressId = create_inc_dec(section, "Rim sound ID", 5, &currentPad->soundRimAddressId);
    soundCupAddressId = create_inc_dec(section, "Cup sound ID", 5, &currentPad->soundCupAddressId);

    soundHeadVolumeDb = create_slider_float(section, NULL, "Head Sound Volume dB", -80., 6., 0.5, &currentPad->soundHeadVolumeDb);
    soundRimVolumeDb = create_slider_float(section, NULL, "Rim Sound Volume dB", -80., 6., 0., &currentPad->soundRimVolumeDb);
    soundCupVolumeDb = create_slider_float(section, NULL, "Cup Sound Volume dB", -80., 6., 0., &currentPad->soundCupVolumeDb);

    /*Create a root page*/

    root_page = lv_menu_page_create(menu, "Settings");
    lv_obj_set_style_pad_hor(root_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    section = lv_menu_section_create(root_page);
    cont = create_text(section, LV_SYMBOL_SETTINGS, "Sensitivity", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_mechanics_page);
    cont = create_text(section, LV_SYMBOL_AUDIO, "Notes", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_note_page);
    cont = create_text(section, LV_SYMBOL_BELL, "Sounds", LV_MENU_ITEM_BUILDER_VARIANT_1);
	lv_menu_set_load_page_event(menu, cont, sub_sound_page);

	lv_obj_t * obj = lv_menu_cont_create(root_page);
	saveBtn = lv_btn_create(obj);
	lv_obj_add_event(saveBtn, save_btn_cb, LV_EVENT_CLICKED, NULL);
	lv_obj_t * btnSaveString = lv_label_create(saveBtn);
	lv_label_set_text(btnSaveString, "Save");
	lv_obj_center(btnSaveString);

	obj = lv_menu_cont_create(root_page);
	cancelBtn = lv_btn_create(obj);
	lv_obj_add_event(cancelBtn, cancel_btn_cb, LV_EVENT_ALL, NULL);
	lv_obj_t *btnCancelString = lv_label_create(cancelBtn);
	lv_label_set_text(btnCancelString, "Cancel");
	lv_obj_center(btnCancelString);

	if(*_padState == PAD_NOT_CHANGED){
		lv_obj_add_flag(saveBtn, LV_OBJ_FLAG_HIDDEN);
		lv_obj_add_flag(cancelBtn, LV_OBJ_FLAG_HIDDEN);
	}

    lv_menu_set_sidebar_page(menu, root_page);

    lv_obj_send_event(lv_obj_get_child(lv_obj_get_child(lv_menu_get_cur_sidebar_page(menu), 0), 0), LV_EVENT_CLICKED, NULL);
}

#endif
