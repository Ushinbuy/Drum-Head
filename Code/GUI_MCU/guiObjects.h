#ifndef GUI_OBJECTS_H_
#define GUI_OBJECTS_H_

typedef struct {
	lv_obj_t * sliderObj;
	lv_obj_t * sliderValueText;
	void * addressPadParameter;
} SliderWithText;

typedef struct {
	lv_obj_t * incButton;
	lv_obj_t * decButton;
	lv_obj_t * valueText;
	void * addressPadParameter;
	uint8_t maxValue;
} IdButtonsObj;

#endif /* GUI_OBJECTS_H_ */
