/*
 * DigitalOut.cpp
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

#include <DigitalOut.h>

/**
 * Construtor
 *
 * port					Porta que contém o pino a ser inicializado
 * pin					Número do pino
 *
 */
DigitalOut::DigitalOut(GPIO_TypeDef* port, uint32_t pin) {
	error = true;

	this->port = port;
	this->pin = pin;

	config.Pin = this->pin;
	config.Mode = GPIO_MODE_OUTPUT_PP;
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

	//Inicializa o pino em nível lógico baixo
	HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_RESET);

	//Inicializa o pino
	HAL_GPIO_Init(this->port, &config);

	error = false;
}

/**
 * Destrutor
 */
DigitalOut::~DigitalOut() {
	if (!error) {
		HAL_GPIO_DeInit(this->port, this->pin);
		error = true;
	}
}

/**
 * Escreve o valor no pino
 *
 * value				Se false, nível lógico baixo no pino. Se true, nível lógico alto.
 *
 */
void DigitalOut::write(bool state) {
	if (!error) {
		if (state) {
			HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_RESET);
		}
	}
}

/**
 * Seta o valor do pino de saída
 */
void DigitalOut::set() {
	if (!error) {
		HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_SET);
	}
}

/**
 * Reseta o valor do pino de saída
 */
void DigitalOut::reset() {
	if (!error) {
		HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_RESET);
	}
}

/**
 * Alterna o estado do pino de saída
 */
void DigitalOut::toggle() {
	if (!error) {
		HAL_GPIO_TogglePin(this->port, this->pin);
	}
}

/**
 * Retorna false se nível lógico baixo ou true se nível lógico alto na saída.
 *
 */
bool DigitalOut::read() {
	if (!error) {
		if (HAL_GPIO_ReadPin(this->port, this->pin)) {
			return true;
		}
	}

	return false;
}

/**
 *	Retorna true se houver algum erro
 *
 */
bool DigitalOut::getError() {
	return error;
}
