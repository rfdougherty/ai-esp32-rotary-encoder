// based on https://github.com/marcmerlin/IoTuz code - extracted and modified Encoder code
//
//

#include "esp32Encoder.h"

void IRAM_ATTR esp32Encoder::readEncoder_ISR()
{
	portENTER_CRITICAL_ISR(&(this->mux));
	if (this->isEnabled) {
		this->old_AB <<= 2;                   //remember previous state
		int8_t ENC_PORT = ((digitalRead(this->BPin)) ? (1 << 1) : 0)
                        | ((digitalRead(this->APin)) ? (1 << 0) : 0);
		this->old_AB |= ( ENC_PORT & 0x03 );  //add current state
		this->encoder0Pos += ( this->enc_states[( this->old_AB & 0x0f )]);
		if (this->encoder0Pos > (this->_maxEncoderValue))
			this->encoder0Pos = this->_circleValues ? this->_minEncoderValue : this->_maxEncoderValue;
		if (this->encoder0Pos < (this->_minEncoderValue))
			this->encoder0Pos = this->_circleValues ? this->_maxEncoderValue : this->_minEncoderValue;
	}
	portEXIT_CRITICAL_ISR(&(this->mux));
}


esp32Encoder::esp32Encoder(uint8_t APin, uint8_t BPin, uint8_t buttonPin, uint8_t steps)
{
	this->old_AB = 0;
	this->APin = APin;
	this->BPin = BPin;
	this->ButtonPin = buttonPin;
	this->Steps = steps;
	pinMode(this->APin, INPUT_PULLUP);
	pinMode(this->BPin, INPUT_PULLUP);
}

void esp32Encoder::setBoundaries(int16_t minEncoderValue, int16_t maxEncoderValue, bool circleValues)
{
	this->_minEncoderValue = minEncoderValue * this->Steps;
	this->_maxEncoderValue = maxEncoderValue * this->Steps;
	this->_circleValues = circleValues;
}

int16_t esp32Encoder::readEncoder()
{
	return (this->encoder0Pos / this->Steps);
}

int16_t esp32Encoder::encoderChanged() {
	int16_t _encoder0Pos = readEncoder();

	int16_t encoder0Diff = _encoder0Pos - this->lastReadEncoder0Pos;

	this->lastReadEncoder0Pos = _encoder0Pos;
	return encoder0Diff;
}

void esp32Encoder::setup(void (*ISR_callback)(void))
{
	attachInterrupt(digitalPinToInterrupt(this->APin), ISR_callback, CHANGE);
	attachInterrupt(digitalPinToInterrupt(this->BPin), ISR_callback, CHANGE);
}


void esp32Encoder::begin()
{
	this->lastReadEncoder0Pos = 0;
	// Initialize rotary encoder reading and decoding
	this->previous_butt_state = 0;
	if (this->ButtonPin >= 0) {
		pinMode(this->ButtonPin, INPUT_PULLUP);
	}
}

ButtonState esp32Encoder::currentButtonState()
{
	if (!this->isEnabled) {
		return BUT_DISABLED;
	}

	uint8_t butt_state = !digitalRead(this->ButtonPin);


	if (butt_state && !this->previous_butt_state)
	{
		this->previous_butt_state = true;
		return BUT_PUSHED;
	}
	if (!butt_state && this->previous_butt_state)
	{
		this->previous_butt_state = false;
		return BUT_RELEASED;
	}
	return (butt_state ? BUT_DOWN : BUT_UP);
}

void esp32Encoder::reset(int16_t newValue_) {
	newValue_ = newValue_ * this->Steps;
	this->encoder0Pos = newValue_;
	if (this->encoder0Pos > this->_maxEncoderValue)
        this->encoder0Pos = this->_circleValues ? this->_minEncoderValue : this->_maxEncoderValue;
	if (this->encoder0Pos < this->_minEncoderValue)
        this->encoder0Pos = this->_circleValues ? this->_maxEncoderValue : this->_minEncoderValue;
}

void esp32Encoder::enable() {
	this->isEnabled = true;
}
void esp32Encoder::disable() {
	this->isEnabled = false;
}
