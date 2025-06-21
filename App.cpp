/**
 * @file           : App.cpp
 * @author         : Foundary
 * @brief          : None
 * @date           : 2025/4/3 23:04
 * @lastEditor     : Star_Plucking
 * @lastEditTime   : 2025/4/4
 */

#include <ConnectionGuard.h>
#include <sheriff_os.h>
#include <units.h>

USE_CONNECTION_GUARD;

using namespace units::literals;

int cdc = 0;

// 这个线程需要给予最高等级的保护，这个线程就是一切
CREATE_THREAD_STATIC(taskcdc, 1024, NULL, 5) {
    for (;;) {
        ++cdc;
        os::Sleep(10_ms);
    }
}

namespace InitAllDevice {
// ----- FDCAN1 <==> wheel + yaw
// ----- FDCAN2 <==> gimbal board(USE CAN-FD)
// ----- FDCAN3 <==> super cap + rudder + feeder
/**
 * @name can错误回调函数
 * @param hfdcan fdcan句柄
 * @param _ 错误类型
 */
void CanErrorCb(FDCAN_HandleTypeDef* hfdcan, uint32_t _) {
    // ---- 处理can芯片掉电但是单片机仍在工作的情况
    // ---- 锁死程序，不断尝试重启
    if ((_ & FDCAN_IT_BUS_OFF) != RESET) {
        FDCAN_ProtocolStatusTypeDef protocolStatus = {};
        HAL_FDCAN_GetProtocolStatus(hfdcan, &protocolStatus);
        if (protocolStatus.BusOff) {
            CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
        }
    }
}

void FdCanInit(FDCAN_HandleTypeDef* hfdcan) {
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x0000;
    sFilterConfig.FilterID2 = 0x0000;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE) != HAL_OK) {
        Error_Handler();
    }

    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x1FFFFFFF;
    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE) != HAL_OK) {
        Error_Handler();
    }

    // 板通使用 CAN FD
    if (hfdcan == os::Can::Instance(FDCAN2).GetHandlePtr()) {
        HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, hfdcan->Init.DataPrescaler * hfdcan->Init.DataTimeSeg1, 0);
        HAL_FDCAN_EnableTxDelayCompensation(hfdcan);
    }

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_TX_EVT_FIFO_NEW_DATA, 0) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
        Error_Handler();
    }
}

void InitCan() {
    HAL_FDCAN_RegisterErrorStatusCallback(os::Can::Instance(FDCAN1).GetHandlePtr(), CanErrorCb);
    HAL_FDCAN_RegisterErrorStatusCallback(os::Can::Instance(FDCAN2).GetHandlePtr(), CanErrorCb);
    HAL_FDCAN_RegisterErrorStatusCallback(os::Can::Instance(FDCAN3).GetHandlePtr(), CanErrorCb);
    FdCanInit(os::Can::Instance(FDCAN1).GetHandlePtr());
    FdCanInit(os::Can::Instance(FDCAN2).GetHandlePtr());
    FdCanInit(os::Can::Instance(FDCAN3).GetHandlePtr());
}

/**
 *
 * @tparam buffer UART接收缓冲区地址
 * @tparam size UART接收缓冲区大小
 * @param huart UART句柄
 */
template <uint8_t* buffer, size_t size>
void UartErrorHandler(UART_HandleTypeDef* huart) {
    // 校验位错误，硬件问题一般不会出现多次，忽略即可
    if (huart->ErrorCode & HAL_UART_ERROR_PE) {
        __HAL_UART_CLEAR_PEFLAG(huart);
    }
    // 帧错误
    if (huart->ErrorCode & HAL_UART_ERROR_FE) {
        __HAL_UART_CLEAR_FEFLAG(huart);
    }
    if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
        __HAL_UART_CLEAR_OREFLAG(huart);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(huart, buffer, size);
}

// ------ UART2 <==> referee
constexpr auto kUart2RxBufferLen = 500;
uint8_t uart2_rx_buffer[kUart2RxBufferLen];

void Uart2RxCb(UART_HandleTypeDef* huart, uint16_t len) {
    os::Uart::Instance(USART2).on_receive.emit(uart2_rx_buffer, len);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, uart2_rx_buffer, kUart2RxBufferLen);
}

// ------ UART1 <==> power board
constexpr auto kUart1RxBufferLen = 20;
uint8_t uart1_rx_buffer[kUart1RxBufferLen];

void Uart1RxCb(UART_HandleTypeDef* huart, uint16_t len) {
    os::Uart::Instance(USART1).on_receive.emit(uart1_rx_buffer, len);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, uart1_rx_buffer, kUart1RxBufferLen);
}

void InitUart() {
    HAL_UART_RegisterCallback(os::Uart::Instance(USART2).GetHandlePtr(), HAL_UART_ERROR_CB_ID,
                              UartErrorHandler<uart2_rx_buffer, kUart2RxBufferLen>);
    HAL_UART_RegisterRxEventCallback(os::Uart::Instance(USART2).GetHandlePtr(), Uart2RxCb);
    HAL_UARTEx_ReceiveToIdle_DMA(os::Uart::Instance(USART2).GetHandlePtr(), uart2_rx_buffer, kUart2RxBufferLen);
    HAL_UART_RegisterCallback(os::Uart::Instance(USART1).GetHandlePtr(), HAL_UART_ERROR_CB_ID,
                              UartErrorHandler<uart1_rx_buffer, kUart1RxBufferLen>);
    HAL_UART_RegisterRxEventCallback(os::Uart::Instance(USART1).GetHandlePtr(), Uart1RxCb);
    HAL_UARTEx_ReceiveToIdle_DMA(os::Uart::Instance(USART1).GetHandlePtr(), uart1_rx_buffer, kUart1RxBufferLen);
}
} // namespace InitAllDevice

InitLate(__init) {
    InitAllDevice::InitCan();
    InitAllDevice::InitUart();
}