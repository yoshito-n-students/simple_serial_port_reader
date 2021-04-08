#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <simple_serial_port_reader/simple_serial_port_reader.hpp>

PLUGINLIB_EXPORT_CLASS(simple_serial_port_reader::SimpleSerialPortReader, nodelet::Nodelet);