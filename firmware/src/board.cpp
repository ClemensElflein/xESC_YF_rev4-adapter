// Board initialization and logger instantiation
#include "board.hpp"

#ifdef PROTO_DEBUG
// Create an IODeviceWrapper around the Uart Peripheral we want to use
modm::IODeviceWrapper<Board::proto_uart::Uart, modm::IOBuffer::BlockIfFull> Board::LoggerDevice;

// Set all four logger streams to use the UART
modm::log::Logger modm::log::debug(Board::LoggerDevice);
modm::log::Logger modm::log::info(Board::LoggerDevice);
modm::log::Logger modm::log::warning(Board::LoggerDevice);
modm::log::Logger modm::log::error(Board::LoggerDevice);
#endif
