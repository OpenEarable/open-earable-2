#include "button_manager.h"
#include "Button.h"

int button_pressed(enum button_pin_names pin, bool * pressed) {
	switch (pin) {
		case BUTTON_EARABLE: //BUTTON_PLAY_PAUSE:
			*pressed = (earable_btn.getState() == BUTTON_PRESS);
			return 0;
		/*case BUTTON_VOLUME_DOWN:
			return volume_down_btn.getState() == BUTTON_PRESS;
		case BUTTON_VOLUME_UP:
			return volume_up_btn.getState() == BUTTON_PRESS;
		case BUTTON_4:
			return four_btn.getState() == BUTTON_PRESS;
		case BUTTON_5:
			return five_btn.getState() == BUTTON_PRESS;*/
		default:
			*pressed = false;
			return 0;
	}
}