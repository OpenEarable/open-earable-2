#include "button_manager.h"
#include "Button.h"

bool button_pressed(enum button_pin_names pin) {
	switch (pin) {
		case BUTTON_PLAY_PAUSE:
			return earable_btn.getState() == BUTTON_PRESS;
		case BUTTON_VOLUME_DOWN:
			return volume_down_btn.getState() == BUTTON_PRESS;
		case BUTTON_VOLUME_UP:
			return volume_up_btn.getState() == BUTTON_PRESS;
		case BUTTON_4:
			return four_btn.getState() == BUTTON_PRESS;
		case BUTTON_5:
			return five_btn.getState() == BUTTON_PRESS;
		default:
			return false;
	}
}