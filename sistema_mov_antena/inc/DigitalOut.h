/*
 * DigitalOut.h
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

#ifndef DIGITALOUT_H_
#define DIGITALOUT_H_

#include "stm32f4xx_hal.h"
#include "Pins.h"

class DigitalOut {
private:
	uint32_t pin;					//Pino do porta
	GPIO_TypeDef* port;				//Porta a ser utilizada
	GPIO_InitTypeDef config;		//Configuração do pino
	bool error;						//False se nenhum erro ocorreu

public:
	/**
	 * Construtor
	 *
	 * port					Porta que contém o pino a ser inicializado
	 * pin					Número do pino
	 *
	 */
	DigitalOut(GPIO_TypeDef* port, uint32_t pin);

	/**
	 * Destrutor
	 */
	~DigitalOut();

	/**
	 * Escreve o valor no pino
	 *
	 * value				Se false, nível lógico baixo no pino. Se true, nível lógico alto.
	 *
	 */
	void write(bool state);

	/**
	 * Seta o valor do pino de saída
	 */
	void set();

	/**
	 * Reseta o valor do pino de saída
	 */
	void reset();

	/**
	 * Alterna o estado do pino de saída
	 */
	void toggle();

	/**
	 * Retorna false se nível lógico baixo ou true se nível lógico alto na saída.
	 *
	 */
	bool read();

	/**
	 *	Retorna true se houver algum erro
	 *
	 */
	bool getError();
};

#endif /* DIGITALOUT_H_ */
