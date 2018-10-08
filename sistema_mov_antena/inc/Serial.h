/*
 * Serial.h
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

#ifndef SERIAL_H_
#define SERIAL_H_

#include "stm32f4xx_hal.h"

#include <string>
#include <stdarg.h>
#include "CircularBuffer.h"

class Serial {
	private:
		UART_HandleTypeDef huart;				//Handler da porta serial
		bool error;								//False se nenhum erro ocorreu
		CircularBuffer<uint8_t>* rxBuffer;

	public:
		/**
		 * Construtor
		 *
		 * instance				Porta da serial
		 * baud					Baud rate da serial
		 */
		Serial(USART_TypeDef* instance, uint32_t baud);

		/**
		 * Construtor
		 *
		 * instance				Instância da serial
		 * baud					Baud rate da serial
		 * bufferSize			Tamanho do buffer de recepção
		 */
		Serial(USART_TypeDef* instance, uint32_t baud, uint16_t bufferSize);

		/**
		 * Destrutor
		 */
		~Serial();

		/**
		 * Callback da interrupção de recepção
		 */
		void interruptCallback();

		/**
		 * Envia bytes pela serial
		 *
		 * data					Array com os dados a serem enviados
		 * length				Quantidade de bytes do array data a serem enviados
		 */
		void write(char *data, uint16_t length);

		/**
		 * Envia uma string pela serial, ou seja, envia todos os bytes do array data,
		 * em ordem, até encontrar um '\0'
		 *
		 * data					A string a ser enviada
		 */
		void print(std::string data);

		void print(const char *fmt, ...);

		/**
		 * Envia uma string pela serial, ou seja, envia todos os bytes do array data,
		 * em ordem, e adiciona uma quebra de linha ao fim
		 *
		 * data					A string a ser enviada
		 */
		void println(std::string data);

		void println(const char *fmt, ...);

		/**
		 * Retorna o número de bytes contidos no buffer de recepção
		 */
		size_t available();

		/**
		 * Lê do buffer de recepção o número de bytes especificado no array data
		 *
		 * data					O array para escrita dos dados
		 * length				Quantidade de bytes a serem escritos no array
		 */
		void read(uint8_t* data, uint16_t length);

		/**
		 * Retorna o primeiro byte contido no buffer de recepção sem retirá-lo
		 * do mesmo
		 *
		 * data					O endereço onde o byte deve ser escrito
		 */
		void peek(uint8_t* data);

		/**
		 * Retorna o primeiro byte contido no buffer
		 *
		 * data					O endereço onde o byte deve ser escrito
		 */
		void getChar(uint8_t* data);

		/**
		 * Retorna false se a porta estiver funcionando e true se tiver ocorrido algum erro
		 */
		bool getError();
};

#endif /* SERIAL_H_ */
