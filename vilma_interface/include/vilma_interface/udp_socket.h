/**
 *
 * @brief This  send/receive data by udp using asynchronoues function!!
 * @author olmer Garcia olmerg@gmail.com
 *
 * Copyright 2013 Olmer Garcia Bedoya
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef UDP_SOCKET_H_
#define UDP_SOCKET_H_
#include <boost/thread/mutex.hpp>
// #include "boost/assign/std/vector.hpp"
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <sstream>

using boost::asio::deadline_timer;
using boost::asio::ip::udp;
namespace microautobox
{
    /**
     * function to convert <type> to str
     */
    template <typename T>
    std::string toString(const T &value)
    {
        std::ostringstream oss;
        oss << value;
        return oss.str();
    }

    /**
     * class that send/receive or client request data by udp using asynchronoues function and timeout!!
     *
     * it is really necessary to have two sockets in the linux pc?
     * it could be better to look for a protocol and implement it in matlab?
     * read http://stackoverflow.com/questions/276058/what-is-the-optimal-size-of-a-udp-packet-for-maximum-throughput
     *    SCTP or DCCP
     *
     */
    class udp_socket
    {
        // private:  mutable boost::mutex mtx_;
        boost::asio::io_service io_service_;
        udp::socket socket_server;
        udp::endpoint recieved_endpoint_;
        udp::endpoint tmp_endpoint_;
        boost::shared_ptr<boost::asio::deadline_timer> deadline_;
        unsigned short port_client;
        boost::posix_time::time_duration timeout;
        enum
        {
            max_length = 1024
        };
        unsigned char data_rx[max_length];
        int length_tx;
        int length_rx;
        bool _message_received, _timeout_triggered, _message_sent, _clientmode;
        int errores;
        short port;

    public:
        std::string message;
        udp_socket() : // io_service_(io_service),
                       socket_server(io_service_),
                       deadline_(new boost::asio::deadline_timer(io_service_))
        {
            port = 5001;
            errores = 0;
            timeout = boost::posix_time::millisec(9);
            length_tx = 0;
            length_rx = 0;
        }
        /**
         *constructor of the class
         * @param port			port of the server in the embedded pc
         * @param port_client  port of the microautobox II
         * @param IP_client    IP of the microautobox II
         * @param timeout       time out to make a request
         */
        udp_socket(short port, short port_client, std::string IP_client, boost::posix_time::time_duration _timeout) : // io_service_(io_service),
                                                                                                                      socket_server(io_service_),
                                                                                                                      //	signals_(io_service_),
                                                                                                                      // myresolver(io_service),
                                                                                                                      deadline_(new boost::asio::deadline_timer(io_service_))
        {
            /*	// Register to handle the signals that indicate when the server should exit.
             * // It is safe to register for the same signal multiple times in a program,
             * // provided all registration for the specified signal is made through Asio.
             * signals_.add(SIGINT);
             * signals_.add(SIGTERM);
             * #if defined(SIGQUIT)
             * signals_.add(SIGQUIT);
             * #endif // defined(SIGQUIT)
             * signals_.async_wait(boost::bind(&maudp::handle_stop, this));
             */

            configure(port, port_client, IP_client, _timeout);
        };
        /**
         *configure the socket
         * @param port			port of the server in the embedded pc
         * @param port_client  port of the microautobox II
         * @param IP_client    IP of the microautobox II
         * @param _timeout       time out to make a request
         * @param size_y        number of sensor variables of MA (default 13)
         * @param size_u		number of commands to MA (default 10)
         */
        bool configure(short port, short port_client, std::string IP_client, boost::posix_time::time_duration _timeout)
        {
            this->port = port;
            recieved_endpoint_.address(boost::asio::ip::address::from_string(IP_client));
            recieved_endpoint_.port(port_client);
            timeout = _timeout;
            if (~socket_server.is_open())
                return open_udp_socket();

            return false;
        }
        bool open_udp_socket()
        {
            if (socket_server.is_open() == false)
            {
                socket_server.open(udp::v4());
                boost::system::error_code ec;
                socket_server.bind(udp::endpoint(udp::v4(), this->port), ec);

                if (ec)
                {
                    message.clear();
                    message.append("ERROR. Error binding the socket ");
                    message.append(ec.message()); // ROS_INFO("error to receive data %s ",error.message());
                    message.append("\n");
                    return false;
                }
                errores = 0;
                length_tx = 0;
                length_rx = 0;
                // like the package are small reduce the buffer
                boost::asio::socket_base::receive_buffer_size option(256);
                socket_server.set_option(option);
                // http://stackoverflow.com/questions/2769555/boost-asio-udp-retrieve-last-packet-in-socket-buffer
                // http://stackoverflow.com/questions/2031109/understanding-set-getsockopt-so-sndbuf
                //  minimum is 1024 (doubled by the kernel to 2048)  http://man7.org/linux/man-pages/man7/socket.7.html /SO_RCVBUF /SO_SNDBUF
                boost::asio::socket_base::send_buffer_size option1(1024);
                socket_server.set_option(option1);

                socket_server.set_option(option);
                // http://man7.org/linux/man-pages/man7/socket.7.html problem with messages not sent
                boost::asio::socket_base::linger option2(true, 3);
                socket_server.set_option(option2);

                // do not route, just local network
                // boost::asio::socket_base::do_not_route option3(true);
                // socket.set_option(option3);

                // socket_server.set_option(options);
                return true;
            }
            else
            {
                errores = 0;
                length_tx = 0;
                length_rx = 0;
                return true;
            }
        }

        /**
         *destrutor
         */
        //       virtual ~udp_socket();
        /**
         * synchronous function to execute the send then receive to the other terminal with timeout
         * @return number of bytes received (-1 is timeout)
         */

        int clientrequest(unsigned char *data_rx_ptr, unsigned char *data_tx, int _length_tx, std::string &_msg)
        {
            length_tx = _length_tx;
            io_service_.reset();
            message.clear();
            _message_received = false;
            _timeout_triggered = false;
            _message_sent = false;
            _clientmode = true;
            length_rx = -1;
            deadline_->expires_at(deadline_timer::traits_type::now() + timeout);
            deadline_->async_wait(boost::bind(&udp_socket::timer_deadline, this, boost::asio::placeholders::error));
            int a = cleanrxbuffer();
            _clientmode = true;
            if (a > 0)
            {
                message.append("WARNING [RX] buffer with   ");
                message.append(toString<int>(a));
                message.append(" bytes \n");
            }

            socket_server.async_send_to(
                boost::asio::buffer(data_tx, length_tx), recieved_endpoint_, boost::bind(&udp_socket::handle_tx, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

            while (io_service_.run_one())
            {
                /* if (_message_sent)
                  {
                      _timeout_triggered=false;
                      _message_sent=false;
                  }*/
                if (_message_sent && _message_received)
                {
                    deadline_->cancel();
                    _timeout_triggered = false;
                    // _message_received=false;
                    //_message_sent=false;
                }
                else if (_timeout_triggered)
                {
                    _message_received = false;
                    _message_sent = false;
                    _timeout_triggered = false; // TODO: i do not sure
                    socket_server.cancel();
                    errores++;
                    message.append("ERROR [TIMEOUT] Client Request ");
                    message.append(toString<int>(errores));
                    message.append(toString<bool>(_message_sent));
                    message.append(toString<bool>(_message_received));
                    message.append("\n");
                    if (length_rx < 1)
                        length_rx--;
                    // break;
                }
            }
            io_service_.reset();
            _msg = message;
            for (int i = 0; i < length_rx; i++)
                data_rx_ptr[i] = data_rx[i];

            return length_rx;
        };

        /**
         * synchronous function to execute the receive to the other terminal with timeout
         * @return number of bytes received (-1 is timeout)
         */

        int receive(unsigned char *data_rx_ptr, std::string &_msg)
        {
            io_service_.reset();
            // io_service_.reset();
            message.clear();
            _message_received = false;
            _timeout_triggered = false;
            _message_sent = false;
            _clientmode = false;
            deadline_->expires_at(deadline_timer::traits_type::now() + timeout);
            deadline_->async_wait(boost::bind(&udp_socket::timer_deadline, this, boost::asio::placeholders::error));

            socket_server.async_receive_from(
                boost::asio::buffer(data_rx, max_length),
                tmp_endpoint_, // here it is not posible to use the received_endpoint  tmp_endpoint_?
                boost::bind(&udp_socket::handle_receive_from, this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));

            while (io_service_.run_one())
            {
                if (_message_received)
                {
                    deadline_->cancel();
                    _timeout_triggered = false;
                    // _message_received=false;
                }
                else if (_timeout_triggered)
                {
                    _message_received = false;
                    //  _timeout_triggered=false;
                    socket_server.cancel();
                    errores++;
                    message.append("ERROR [TIMEOUT] Receive ");
                    message.append(toString<int>(errores));
                    message.append(" \n");
                    length_rx = -1;
                    // break;
                }
            }
            io_service_.reset();
            _msg = message;
            for (int i = 0; i < length_rx; i++)
                data_rx_ptr[i] = data_rx[i];
            return length_rx;
        };
        /**
         * synchronous function to execute the send to the other terminal with timeout
         * the buffer of receive data is clean because is the buffer has data is from another request to other terminal
         * @return message of the request
         */

        int send(unsigned char *data_tx, int _length_tx, std::string &_msg)
        {
            length_tx = _length_tx;
            io_service_.reset();
            message.clear();
            _message_received = false;
            _timeout_triggered = false;
            _message_sent = false;
            _clientmode = false;

            deadline_->expires_at(deadline_timer::traits_type::now() + timeout);
            deadline_->async_wait(boost::bind(&udp_socket::timer_deadline, this, boost::asio::placeholders::error));
            int a = cleanrxbuffer();

            if (a > 0)
            {
                message.append("WARNING. [RX] buffer with   ");
                message.append(toString<int>(a));
                message.append(" bytes \n");
            }

            socket_server.async_send_to(
                boost::asio::buffer(data_tx, length_tx), recieved_endpoint_, boost::bind(&udp_socket::handle_tx, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

            while (io_service_.run_one())
            {
                if (_message_sent)
                {
                    deadline_->cancel();
                    _timeout_triggered = false;
                    // _message_sent=false;
                }

                /* if (_message_received)
                 {
                     deadline_->cancel();
                     _timeout_triggered=false;
                     _message_received=false;
                 }*/
                else if (_timeout_triggered)
                {
                    _message_received = false;
                    // _timeout_triggered=false;
                    socket_server.cancel();
                    errores++;
                    message.append("ERROR [TIMEOUT] Send ");
                    message.append(toString<int>(errores));
                    message.append(" \n");
                    length_tx = -1;
                    // break;
                }
            }
            io_service_.reset();
            _msg = message;
            return length_tx;
        };

        int get_errores()
        {
            return errores;
        };
        bool is_open()
        {
            return socket_server.is_open();
        }
        bool close_udp_socket()
        {
            length_tx = 0;
            length_rx = 0;
            if (socket_server.is_open())
            {
                socket_server.cancel();

                boost::system::error_code ec;
                socket_server.shutdown(boost::asio::ip::udp::socket::shutdown_both, ec);
                // socket_server.shutdown(boost::asio::ip::udp::socket::shutdown_receive, ec);
                message.clear();
                if (ec)
                {
                    message.append(ec.message());
                }
                socket_server.close(ec);
                if (ec)
                {

                    message.append("ERROR [CLOSE] ");
                    message.append(ec.message()); // ROS_INFO("error to receive data %s ",error.message());
                    message.append("\n");
                    return false;
                }
                else
                    errores = 0;
            }
            return true;
        };

    private:
        /**
         *@brief asynchronous function to send data (TX) and make the request to receive data.
         * @param ec   sent errors
         * @param length sent bytes
         */
        void handle_tx(const boost::system::error_code &ec, std::size_t length)
        {
            _message_sent = true;
            if (!ec && length == length_tx)
            {
                if (_clientmode)
                {
                    length_rx = 0;
                    socket_server.async_receive_from(
                        boost::asio::buffer(data_rx, max_length),
                        recieved_endpoint_, // here it is necessary to received from the person who send the message, could be used tmp_endpoint_?
                        boost::bind(&udp_socket::handle_receive_from, this,
                                    boost::asio::placeholders::error,
                                    boost::asio::placeholders::bytes_transferred));
                }
                length_tx = length;
            }
            else
            {
                message.append("ERROR [TX] ");
                message.append(ec.message());
                message.append("\n");
                length_rx = 0;
                length_tx = 0;
            }
        };

        /**
         * function that recieve the information from the other program and
         * send the answer to them
         * @param error

         * @param bytes_recvd
         */
        void handle_receive_from(const boost::system::error_code &error, size_t bytes_recvd)
        {
            _message_received = true;
            if (!error)
            {
                length_rx = bytes_recvd;
            }
            else
            {

                message.append("ERROR[RX] ");
                message.append(error.message()); // ROS_INFO("error to receive data %s ",error.message());
                message.append("\n");
                errores++;
                length_rx = 0;
            }
        };

        /**
         * @brief timer function which control the time maximum to sent and receive
         * a data to the MicroAutobox.
         * this function cancel the TX or RX if timeout
         *
         * TODO: implementar alarmas
         */
        void timer_deadline(const boost::system::error_code &error)
        {
            if (error != boost::asio::error::operation_aborted)
            {

                // if (deadline_->expires_at() <= deadline_timer::traits_type::now())
                //{
                _timeout_triggered = true;
                deadline_->expires_at(boost::posix_time::pos_infin);
                //}
            }
        };
        /// The signal_set is used to register for process termination notifications.
        //  boost::asio::signal_set signals_;
        /// Handle a request to stop the server.
        void handle_stop()
        {
            io_service_.stop();
        };

        /// in client mode:
        /// if the last message was not receive by timeout maybe now is available because the server have some problem
        ///@ bug this is a synchronous call to receive which invalidate the timeout
        int cleanrxbuffer()
        {

            boost::asio::socket_base::bytes_readable command(true);
            socket_server.io_control(command);
            std::size_t bytes_readable = command.get();
            if (bytes_readable > 0)
            {
                // std::cout<<"paquete pendiente bytes "<<bytes_readable<<"\n";
                _clientmode = false;
                bytes_readable = socket_server.receive_from(boost::asio::buffer(data_rx, max_length),
                                                            recieved_endpoint_);
                // TODO: it is neccesary to read again until zero?
                // socket_server.io_control(command);
                // std::size_t bytes_readable = command.get();
                // std::cout<<"paquete pendiente bytes "<<bytes_readable<<"\n";
            }
            return bytes_readable;
        };
    };

}

#endif /* UDP_SOCKET_H_ */