/*
 * DIn.cpp
 *
 *  Created on: Nov 2, 2016
 *      Author: Adriano
 */

#include <DigitalIn.h>

DigitalIn::DigitalIn(GPIO_TypeDef* port, uint32_t pin) {
	error = true;

	this->port = port;
	this->pin = pin;

	config.Pin = this->pin;
	config.Mode = GPIO_MODE_INPUT;
	config.Pull = GPIO_NOPULL;
	config.Speed = GPIO_SPEED_FREQ_MEDIUM;

	//Habilita o clock para a porta correta
	if (this->port == GPIOA) {
		__HAL_RCC_GPIOA_CLK_ENABLE();
	} else if (this->port == GPIOB) {
		__HAL_RCC_GPIOB_CLK_ENABLE();
	} else if (this->port == GPIOC) {
		__HAL_RCC_GPIOC_CLK_ENABLE();
	} else if (this->port == GPIOD) {
		__HAL_RCC_GPIOD_CLK_ENABLE();
	} else if (this->port == GPIOH) {
		__HAL_RCC_GPIOH_CLK_ENABLE();
	} else {
		return;
	}

	//Inicializa o pino
	HAL_GPIO_Init(this->port, &config);

	error = false;
}

DigitalIn::~DigitalIn() {
	if (!error) {
		HAL_GPIO_DeInit(this->port, this->pin);
		error = true;
	}
}

bool DigitalIn::get() {
	if (!error) {
		return (HAL_GPIO_ReadPin(this->port, this->pin) == GPIO_PIN_SET);
	}

	return false;
}

void DigitalIn::setPull(uint32_t pull) {
	if (!error) {
		config.Pull = pull;
		HAL_GPIO_Init(this->port, &config);
	}
}

bool DigitalIn::getError() {
	return error;
}
