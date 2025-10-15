// Board initialization and logger instantiation
#include "board.hpp"

#ifdef PROTO_DEBUG
// Create an IODeviceWrapper around the Uart Peripheral we want to use
// This is a global variable definition and must be in a .cpp file to avoid multiple definitions
modm::IODeviceWrapper<Board::proto_uart::Uart, modm::IOBuffer::BlockIfFull> Board::LoggerDevice;

// Set all four logger streams to use the UART
// These are global variable definitions and must be in a .cpp file to avoid multiple definitions
modm::log::Logger modm::log::debug(Board::LoggerDevice);
modm::log::Logger modm::log::info(Board::LoggerDevice);
modm::log::Logger modm::log::warning(Board::LoggerDevice);
modm::log::Logger modm::log::error(Board::LoggerDevice);
#endif
