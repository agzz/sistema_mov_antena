/*
 * Stepper.cpp
 *
 * Copyright (c) 2018 Adriano Zenzen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <Stepper.h>
#include "clock.h"

//Recebe o pino de passo, direção, habilitação e se é para inverter o sentido do motor
Stepper::Stepper(
    		GPIO_TypeDef* stepPort,
			uint32_t stepPinNumber,
			GPIO_TypeDef* dirPort,
			uint32_t dirPinNumber,
			GPIO_TypeDef* enablePort,
			uint32_t enablePinNumber,
			bool invert
	) {

    //Inicializa os pinos
    stepPin = new DigitalOut(stepPort, stepPinNumber);
    dirPin = new DigitalOut(dirPort, dirPinNumber);
    enablePin = new DigitalOut(enablePort, enablePinNumber);

    //Desabilita o driver
    enablePin->set();

    //Configura se o motor é invertido ou não
    inverted = invert;
}

//Recebe o pino de passo, direção e se é para inverter o sentido do motor
Stepper::Stepper(
    		GPIO_TypeDef* stepPort,
			uint32_t stepPinNumber,
			GPIO_TypeDef* dirPort,
			uint32_t dirPinNumber,
			bool invert
	) {

    //Inicializa os pinos
    stepPin = new DigitalOut(stepPort, stepPinNumber);
    dirPin = new DigitalOut(dirPort, dirPinNumber);

    //Configura se o motor é invertido ou não
    inverted = invert;
}

Stepper::~Stepper() {
    delete stepPin;
    delete dirPin;

    if (enablePin != NULL) {
    	delete enablePin;
    }
}

//Executa um passo na direção definida
void Stepper::step() {
	stepPin->set();
	HAL_Delay(2);
    stepPin->reset();
    HAL_Delay(2);
}

//Define a direção de movimento do motor
void Stepper::direction(uint8_t dir) {
    if (dir == CW) {
        if (!inverted) {
            dirPin->reset();
        } else {
        	dirPin->set();
        }
    } else {
        if (!inverted) {
        	dirPin->set();
        } else {
        	dirPin->reset();
        }
    }
}

//Retorna a direção do motor
uint8_t Stepper::getDirection() {
	if (!inverted) {
		if (dirPin->read()) {
			return CCW;
		} else {
			return CW;
		}
	} else {
		if (dirPin->read()) {
			return CW;
		} else {
			return CCW;
		}
	}
}

//Inverte CW e CCW
void Stepper::invertDirection() {
    inverted = !inverted;
}

//Habilita o motor
void Stepper::enable() {
	if (enablePin != NULL) {
		enablePin->reset();
	}
}

//Desabilita o motor
void Stepper::disable() {
	if (enablePin != NULL) {
		enablePin->set();
	}
}

//Retorna verdadeiro se o motor estiver habilitado
bool Stepper::isEnabled() {
	if (enablePin != NULL) {
		return !enablePin->read();
	}

	return false;
}

