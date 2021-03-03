#include <string>

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>
#include <boost/regex.hpp>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/publisher.h>
#include <std_msgs/String.h>

namespace ba = boost::asio;
namespace rp = ros::param;

int main(int argc, char *argv[]) {
  // init ROS
  ros::init(argc, argv, "simple_serial_port_reader");
  ros::NodeHandle nh;

  try {
    // load parameters
    const std::string device = rp::param< std::string >("~device", "/dev/ttyUSB0");
    const int baud_rate = rp::param("~baud_rate", 9600);
    const std::string start_command = rp::param< std::string >("~start_command", "");
    const boost::regex match_expression(rp::param< std::string >("~match_expression", "(.+)\r?\n"));
    const std::string format_expression(rp::param< std::string >("~format_expression", "$1"));
    const std::string stop_command = rp::param< std::string >("~stop_command", "");
    const bool verbose = rp::param("~verbose", false);

    // create the publisher
    ros::Publisher publisher = nh.advertise< std_msgs::String >("formatted", 1);

    // open the serial port
    ba::io_service io_service;
    ba::serial_port serial_port(io_service);
    serial_port.open(device);
    serial_port.set_option(ba::serial_port::baud_rate(baud_rate));

    // write the start command (if any)
    if (!start_command.empty()) {
      ba::write(serial_port, ba::buffer(start_command));
      if (verbose) {
        ROS_INFO_STREAM("wrote as the start command: \"" << start_command << "\"");
      }
    }

    // reading loop
    ba::streambuf buffer;
    while (ros::ok()) {
      // read until the buffer contains the match_expression
      const std::size_t bytes = ba::read_until(serial_port, buffer, match_expression);

      // search matched sequence in the buffer
      const char *const buffer_begin = ba::buffer_cast< const char * >(buffer.data());
      const char *const buffer_end = buffer_begin + bytes;
      if (verbose) {
        ROS_INFO_STREAM("read: \"" << std::string(buffer_begin, bytes) << "\"");
      }
      boost::cmatch match;
      boost::regex_search(buffer_begin, buffer_end, match, match_expression);
      if (verbose) {
        ROS_INFO_STREAM("matched: \"" << match.str() << "\"");
      }

      // format the matched sequence
      std_msgs::String formatted;
      formatted.data = match.format(format_expression);
      if (verbose) {
        ROS_INFO_STREAM("formatted: \"" << formatted.data << "\"");
      }

      // publish the formatted string
      publisher.publish(formatted);

      // consume the buffer processed in this loop
      buffer.consume(bytes);
    }

    // write the stop command (if any)
    if (!stop_command.empty()) {
      ba::write(serial_port, ba::buffer(stop_command));
      if (verbose) {
        ROS_INFO_STREAM("wrote as the stop command: \"" << stop_command << "\"");
      }
    }
  } catch (const std::exception &error) {
    ROS_ERROR_STREAM(error.what());
  }

  return 0;
}