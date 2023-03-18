#ifndef USER_HELLODRUM_INC_HELLODRUMKNOB_HPP_
#define USER_HELLODRUM_INC_HELLODRUMKNOB_HPP_

class HelloDrumKnob
{
public:
	HelloDrumKnob(byte pin1);
	void read() {
		knobValue = analogRead(pin_1);
	}
//	void readMUX() {
//		knobValue = rawValue[pin_1];
//	}

	int knobValue;

private:
	byte pin_1;
};

#endif /* USER_HELLODRUM_INC_HELLODRUMKNOB_HPP_ */
