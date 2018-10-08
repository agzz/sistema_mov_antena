/*
 * Serial.cpp
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

#include <Serial.h>

#include <cstring>

#define USART_NUMBER 3

Serial* handlers[USART_NUMBER] = {0};

Serial::Serial(USART_TypeDef *instance, uint32_t baud) {
	error = true;

	rxBuffer = new (std::nothrow) CircularBuffer<uint8_t>(64);

	huart.Instance = instance;
	huart.Init.BaudRate = baud;
	huart.Init.WordLength = UART_WORDLENGTH_8B;
	huart.Init.StopBits = UART_STOPBITS_1;
	huart.Init.Parity = UART_PARITY_NONE;
	huart.Init.Mode = UART_MODE_TX_RX;
	huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart.Init.OverSampling = UART_OVERSAMPLING_16;
	//huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	//huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (huart.Instance == USART1) {
		// UART clock enable
		__HAL_RCC_USART1_CLK_ENABLE();

		// GPIO clock enable
		__HAL_RCC_GPIOA_CLK_ENABLE();

		// USART1 GPIO Configuration
		// PA9     ------> USART1_TX
		// PA10    ------> USART1_RX
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		// Set callback handler
		handlers[0] = this;

		// USART1 interrupt Init
		HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);

		__HAL_UART_ENABLE_IT(&huart, UART_IT_RXNE);

	} else if (huart.Instance == USART2) {
		// UART clock enable
		__HAL_RCC_USART2_CLK_ENABLE();

		// GPIO clock enable
		__HAL_RCC_GPIOA_CLK_ENABLE();

		// USART2 GPIO Configuration
		// PA2     ------> USART2_TX
		// PA3     ------> USART2_RX
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		// Set callback handler
		handlers[1] = this;

		// USART2 interrupt Init
		HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
		HAL_NVIC_EnableIRQ(USART2_IRQn);

		__HAL_UART_ENABLE_IT(&huart, UART_IT_RXNE);

	} else if (huart.Instance == USART6) {
		// UART clock enable
		__HAL_RCC_USART6_CLK_ENABLE();

		// GPIO clock enable
		__HAL_RCC_GPIOC_CLK_ENABLE();

		// USART6 GPIO Configuration
		// PC6    ------> USART6_TX
		// PC7    ------> USART6_RX
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		// Set callback handler
		handlers[2] = this;

		// USART3 interrupt Init
		HAL_NVIC_SetPriority(USART6_IRQn, 3, 0);
		HAL_NVIC_EnableIRQ(USART6_IRQn);

		__HAL_UART_ENABLE_IT(&huart, UART_IT_RXNE);

	} else {
		return;
	}

	if (HAL_UART_Init(&huart) != HAL_OK) {
		return;
	}

	error = false;
}

Serial::Serial(USART_TypeDef *instance, uint32_t baud, uint16_t bufferSize) {
	error = true;

	if ( (bufferSize <= 0) || (bufferSize >= 512)) {
		return;
	}
	rxBuffer = new (std::nothrow) CircularBuffer<uint8_t>(bufferSize);

	huart.Instance = instance;
	huart.Init.BaudRate = baud;
	huart.Init.WordLength = UART_WORDLENGTH_8B;
	huart.Init.StopBits = UART_STOPBITS_1;
	huart.Init.Parity = UART_PARITY_NONE;
	huart.Init.Mode = UART_MODE_TX_RX;
	huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart.Init.OverSampling = UART_OVERSAMPLING_16;
	//huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	//huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (huart.Instance == USART1) {
		// UART clock enable
		__HAL_RCC_USART1_CLK_ENABLE();

		// GPIO clock enable
		__HAL_RCC_GPIOA_CLK_ENABLE();

		// USART1 GPIO Configuration
		// PA9     ------> USART1_TX
		// PA10    ------> USART1_RX
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		// Set callback handler
		handlers[0] = this;

		// USART2 interrupt Init
		HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);

		__HAL_UART_ENABLE_IT(&huart, UART_IT_RXNE);

	} else if (huart.Instance == USART2) {
		// UART clock enable
		__HAL_RCC_USART2_CLK_ENABLE();

		// GPIO clock enable
		__HAL_RCC_GPIOA_CLK_ENABLE();

		// USART2 GPIO Configuration
		// PA2     ------> USART2_TX
		// PA3     ------> USART2_RX
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		// Set callback handler
		handlers[1] = this;

		// USART2 interrupt Init
		HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
		HAL_NVIC_EnableIRQ(USART2_IRQn);

		__HAL_UART_ENABLE_IT(&huart, UART_IT_RXNE);

	} else if (huart.Instance == USART6) {
		// UART clock enable
		__HAL_RCC_USART6_CLK_ENABLE();

		// GPIO clock enable
		__HAL_RCC_GPIOC_CLK_ENABLE();

		// USART6 GPIO Configuration
		// PC6    ------> USART6_TX
		// PC7    ------> USART6_RX
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		// Set callback handler
		handlers[2] = this;

		// USART3 interrupt Init
		HAL_NVIC_SetPriority(USART6_IRQn, 3, 0);
		HAL_NVIC_EnableIRQ(USART6_IRQn);

		__HAL_UART_ENABLE_IT(&huart, UART_IT_RXNE);
	} else {
		return;
	}

	if (HAL_UART_Init(&huart) != HAL_OK) {
		return;
	}

	error = false;
}

Serial::~Serial() {
	if (!error) {
		if (huart.Instance == USART1) {
			// UART clock disable
			__HAL_RCC_USART1_CLK_DISABLE();

			// USART1 GPIO Configuration
			// PA9     ------> USART1_TX
			// PA10    ------> USART1_RX
			HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);

			// USART1 interrupt DeInit
			HAL_NVIC_DisableIRQ(USART1_IRQn);

			// Clear callback handler
			handlers[0] = 0;

			error = true;

		} else if (huart.Instance == USART2) {
			// UART clock disable
			__HAL_RCC_USART2_CLK_DISABLE();

			// USART2 GPIO Configuration
			// PA2     ------> USART2_TX
			// PA3     ------> USART2_RX
			HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);

			// USART2 interrupt DeInit
			HAL_NVIC_DisableIRQ(USART2_IRQn);

			// Clear callback handler
			handlers[1] = 0;

			error = true;

		} else if (huart.Instance == USART2) {
			// UART clock disable
			__HAL_RCC_USART6_CLK_DISABLE();

			// USART6 GPIO Configuration
			// PC6    ------> USART6_TX
			// PC7    ------> USART6_RX
			HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6 | GPIO_PIN_7);

			// USART6 interrupt DeInit
			HAL_NVIC_DisableIRQ(USART6_IRQn);

			// Clear callback handler
			handlers[2] = 0;

			error = true;

		}
	}
}

void Serial::write(char *data, uint16_t length) {
	if (!error) {
		while (HAL_UART_Transmit(&huart, (uint8_t*) data, length, 0xFFFF) == HAL_BUSY);
	}
}

void Serial::print(std::string data) {
	if (!error) {
		char *cstring = new char[data.length() + 1];
		std::strcpy(cstring, data.c_str());
		write(cstring, data.length());
		delete[] cstring;
	}
}

void Serial::print(const char *fmt, ...) {
	if (!error) {
		char cstring[128];
		va_list arg;
		va_start(arg, fmt);
		vsnprintf(cstring, sizeof(cstring), fmt, arg);
		va_end(arg);
		write(cstring, strlen(cstring));
	}
}

void Serial::println(std::string data) {
	if (!error) {
		data.append("\n");
		char *cstring = new char[data.length() + 1];
		std::strcpy(cstring, data.c_str());
		write(cstring, data.length());
		delete[] cstring;
	}
}

void Serial::println(const char *fmt, ...) {
	if (!error) {
		char cstring[128];
		va_list arg;
		va_start(arg, fmt);
		vsnprintf(cstring, sizeof(cstring), fmt, arg);
		va_end(arg);

		int size = strlen(cstring);
		if (size < 126) {
			cstring[size] = '\n';
			cstring[size + 1] = '\0';
		}
		write(cstring, size + 1);
	}
}

size_t Serial::available() {
	return rxBuffer->size();
}

void Serial::read(uint8_t* data, uint16_t length) {
	if (!error) {
		for (uint16_t i = 0; i < length; i++) {
			data[i] = rxBuffer->get();
		}
	}
}

void Serial::peek(uint8_t* data) {
	if (!error) {
		*data = rxBuffer->peek();
	}
}

void Serial::getChar(uint8_t* data) {
	if (!error) {
		*data = rxBuffer->get();
	}
}

bool Serial::getError() {
	return error;
}

void Serial::interruptCallback() {
	uint32_t flag = 0, source = 0;

	flag = __HAL_UART_GET_FLAG(&huart, UART_FLAG_RXNE);
	source = __HAL_UART_GET_IT_SOURCE(&huart, UART_IT_RXNE);
	if ( (flag != RESET) && (source != RESET) ) {
		rxBuffer->put( (uint8_t)(huart.Instance->DR & (uint16_t)0x00FF) );
	}
}

//Interruption callbacks
extern "C" {
	void USART1_IRQHandler() {
		if (handlers[0] != 0) {
			handlers[0]->interruptCallback();
		}
	}
}

extern "C" {
	void USART2_IRQHandler() {
		if (handlers[1] != 0) {
			handlers[1]->interruptCallback();
		}
	}
}

extern "C" {
	void USART6_IRQHandler() {
		if (handlers[2] != 0) {
			handlers[2]->interruptCallback();
		}
	}
}
