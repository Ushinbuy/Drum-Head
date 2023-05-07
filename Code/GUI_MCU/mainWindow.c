#include "mainWindow.h"
#include "lvgl/examples/lv_examples.h"

void example(){
	lv_obj_t * scr1 = lv_obj_create(NULL);

	lv_obj_t * label = lv_label_create(scr1);
	lv_label_set_text(label, "Hello");

	lv_scr_load(scr1);
}
