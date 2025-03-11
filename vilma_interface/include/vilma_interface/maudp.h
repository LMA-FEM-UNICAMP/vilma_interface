/**
 *
 * @brief This is the library to RX/TX by UDP with the microautobox II
 *http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.68.941&rep=rep1&type=pdf
 *http://www.streamingmedia.com/Articles/Editorial/Featured-Articles/Reliable-UDP-(RUDP)-The-Next-Big-Streaming-Protocol-85316.aspx
 *http://udt.sourceforge.net/software.html
 *https://www.chromium.org/quic
 *http://users.ece.cmu.edu/~koopman/pubs/KoopmanCRCWebinar9May2012.pdf
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
#ifndef MAUDP_H_
#define MAUDP_H_
#include <vector>
// #define DEBUG 1
#include "udp_socket.h"
#include "protocoloc.h"
namespace microautobox
{

    class maudp
    {

    public:
        maudp() {

        };

    public:
        maudp(short port, short port_client, std::string IP_client, boost::posix_time::time_duration _timeout, int size_in, int size_out, int C_PROCESSORTYPE = BIGENDIAN)
        {
            configure(port, port_client, IP_client, _timeout, size_in, size_out);
        };

        bool configure(short port, short port_client, std::string IP_client, boost::posix_time::time_duration _timeout, int size_in, int size_out, int C_PROCESSORTYPE = BIGENDIAN)
        {

            data_tosend.reserve(size_in);
            data_tosend_1.reserve(size_in);
            data_tosend.resize(size_in, 0.0);
            data_tosend_1.resize(size_in, 0.0);

            data_received.reserve(size_out);
            data_received_1.reserve(size_out);
            data_received.resize(size_out, 0.0);
            data_received_1.resize(size_out, 0.0);

            data_tosend_type = 0;
            data_tosend_type_1 = data_tosend_type;
            data_received_type = 0;
            data_received_type_1 = data_received_type;
            tipodato.reserve(4 + size_out + size_in);
            tipodato.resize(4 + size_out + size_in, 9);
            tipodato[0] = 6;
            tipodato[1] = 5;
            tipodato[2] = 5;
            tipodato[3 + size_in + size_out] = 7;
            bytes_udp = size_out * 8 + size_in * 8 + 12;
            trama.reserve(4 + size_out + size_in);
            trama.resize(4 + size_out + size_in, 0);
            buffer_tosend.reserve(bytes_udp);
            buffer_tosend.resize(bytes_udp, 0);
            buffer_received.reserve(bytes_udp);
            buffer_received.resize(bytes_udp, 0);
            send_checksum_error = 0;
            receive_error = 0;
            max_send_error = 5;
            send_error = -1; // max_send_error+1;
            checksum = 0;
            processortype = C_PROCESSORTYPE;
            return _udp_socket.configure(port, port_client, IP_client, _timeout);
        }

    public:
        int matlabdata(unsigned int *buffer_out_ptr, double *data_received_ptr, unsigned short *data_received_type_ptr,
                       unsigned int *buffer_in_ptr, double *data_tosend_ptr, const unsigned short data_tosend_type_)
        {

            // std::vector<unsigned char> buffer_received(bytes_udp);
            //  std::vector<unsigned int> buffer_received_4(buffer_in_ptr,buffer_in_ptr+(int)(bytes_udp/4));

            // prot.maencodetobyte(buffer_received,buffer_received_4);
            maencodetobyte(&buffer_received[0], buffer_in_ptr, buffer_received.size(), processortype);

            data_tosend_type = data_tosend_type_;
            for (unsigned int i = 0; i < data_tosend.size(); i++)
                data_tosend[i] = data_tosend_ptr[i];
            std::string Message;
            make_request(Message);

            // envio a matlab el dato confirmado 100%
            *data_received_type_ptr = data_received_type_1;
            for (unsigned int i = 0; i < data_received_1.size(); i++)
                data_received_ptr[i] = data_received_1[i];
            // envio le buffer
            // prot.maencodetouint32(buffer_received_4,buffer_tosend);
            maencodetouint32(buffer_out_ptr, &buffer_tosend[0], buffer_tosend.size(), processortype);

            // for(unsigned int i=0;i<buffer_received_4.size();i++)
            //     buffer_out_ptr[i]=buffer_received_4[i];
            return 0;
        }

        std::string ma_udp_request(double *data_received_ptr, unsigned short *data_received_type_ptr,
                                   const double *data_tosend_ptr, const unsigned short data_tosend_type_)
        {

            std::string Message;
            data_tosend_type = data_tosend_type_;
            for (unsigned int i = 0; i < data_tosend.size(); i++)
                data_tosend[i] = data_tosend_ptr[i];

            make_request(Message);

            // envio  el dato confirmado 100%
            *data_received_type_ptr = data_received_type_1;
            for (unsigned int i = 0; i < data_received_1.size(); i++)
                data_received_ptr[i] = data_received_1[i];

            return Message;
        }

        std::string ma_udp_server(double *data_received_ptr, unsigned short *data_received_type_ptr,
                                  const double *data_tosend_ptr, const unsigned short data_tosend_type_)
        {
            std::string Message, msg;
            data_tosend_type = data_tosend_type_;
            for (unsigned int i = 0; i < data_tosend.size(); i++)
                data_tosend[i] = data_tosend_ptr[i];

            int bytes_recvd = _udp_socket.receive(&buffer_received[0], Message);
            msg.append(Message);
            if (bytes_recvd > 0)
            { // just if data is received process them
                this->processbuffer(bytes_recvd);
                this->send_data();
                bytes_recvd = _udp_socket.send(&buffer_tosend[0], buffer_tosend.size(), Message);
                msg.append(Message);
            }
            else
            {
                receive_error++;
                send_error++;
            }

            // envio  el dato confirmado 100%
            *data_received_type_ptr = data_received_type_1;
            for (unsigned int i = 0; i < data_received_1.size(); i++)
                data_received_ptr[i] = data_received_1[i];
            return msg;
        }

        void tcp_route(unsigned char *buffer_tosend_ptr, unsigned int &buffer_tosend_size, unsigned char *buffer_received_ptr, unsigned int &buffer_received_size)
        {

            //  std::vector<unsigned char> buffer_tcp_received(buffer_received_ptr,buffer_received_ptr+buffer_received_size);
            std::string Message;
            int ii = 1;
            double tmp = 0;
            buffer_tosend_size = data_received.size() * 8 + 2;

            int i = madecode(&data_tosend[0], &buffer_received_ptr[2], &tipodato[3], data_tosend.size(), processortype);
            tmp = (double)data_tosend_type;
            i = madecode(&tmp, &buffer_received_ptr[0], &tipodato[1], ii, processortype);
            make_request(Message);
#ifdef DEBUG
            std::cout << Message;
#endif
            tmp = (double)data_received_type_1;
            i = maencode(&buffer_tosend_ptr[2], &tmp, &tipodato[1], ii, processortype);

            i = maencode(&buffer_tosend_ptr[2], &data_received_1[0], &tipodato[3], data_received_1.size(), processortype);
        }

        int get_udp_bytes()
        {
            return bytes_udp;
        };
        int get_send_error()
        {
            return send_error;
        }

        int get_receive_error()
        {
            return receive_error;
        }

        int get_udp_error()
        {
            return _udp_socket.get_errores();
        }

        bool is_open()
        {
            return _udp_socket.is_open();
        }
        bool open_udp_socket()
        {
            return _udp_socket.open_udp_socket();
        }

        std::string close_udp_socket()
        {
            /* if i close the socket i get an error
            if(_udp_socket.close_udp_socket()){
                std::string message;
                message.append("the udp socket is closed \n ");
                return message;
            }

            else{

                return _udp_socket.message;
            }*/
            return "ok";
        }

    private:
        void make_request(std::string &message)
        {
            // initial condition
            if (send_error == -1)
            {
                data_tosend_1 = data_tosend;
                data_tosend_type_1 = data_tosend_type;
                send_error = 0;
                send_checksum_error = 0;
            }
            // end initial condition
            send_data();
            int bytes_recvd = _udp_socket.clientrequest(&buffer_received[0], &buffer_tosend[0], buffer_tosend.size(), message);
            //@todo: i think we can generate an alarm if(bytes_recvd!=buffer_received.size())?
            if (bytes_recvd != buffer_received.size() && bytes_recvd > 0)
            {
                message.append("ERROR. RX buffer with  wrong length of bytes: ");
                message.append(toString<int>(bytes_recvd));
                message.append(" bytes \n");
                receive_error++; // error por longitud
                if (bytes_recvd == -1)
                    send_error++; //@bug no estoy seguro de si deba sumar, porque si llego no debo sumar
            }
            else
                this->processbuffer(bytes_recvd);
        }

        int processbuffer(int bytes_recvd)
        {
            // std::vector<unsigned char> v(buffer_received_ptr, buffer_received_ptr + bytes_recvd);
            if (bytes_recvd == bytes_udp)
            {
                // checksum of the buffer received except the las 4 bytes
                checksum = crc_msb(&buffer_received[0], bytes_recvd - 4, GEN_ANSI_MSB);
                // convert to double the buffer received
                int i = madecode(&trama[0], &buffer_received[0], &tipodato[0], trama.size(), processortype);
#ifdef DEBUG
                std::cout << "datos recibidos \n";
                for (int i = 0; i < trama.size(); i++)
                    std::cout << (double)trama[i] << ",";
                std::cout << trama.size() << "\n";

                std::cout << "checksum " << checksum << ":" << (unsigned int)trama[trama.size() - 1] << "\n";
#endif

                // compare checksum
                if (checksum == (unsigned int)trama[trama.size() - 1])
                {
                    //   std::cout<<"trama0 "<<( int)trama[0]<<"\n";

                    // copy the data received
                    std::copy(trama.begin() + 3, trama.begin() + 3 + data_received.size(), data_received.begin());
                    data_received_type = (unsigned short)trama[1];
                    if (trama[0] == 0)
                    { // answer from the server confirmed the data is the same
                        data_received_1 = data_received;
                        data_received_type_1 = data_received_type;
                        receive_error = 0;
                    }
                    else
                    {
                        receive_error++; // error del otro computador
                    }

                    send_checksum_error = 0;
                    /// TODO: will be better to test bytes and not values, taking account that can be faster
                    /// and like is used in different arquitecture (PowerPPC , x86, 64 bits) the double can be different??

                    // compare the trama with the data_to_send_1 (confirmar los datos recibidos con los enviados en la ultima interracion)
                    if (data_tosend_type_1 != (unsigned short)trama[2])
                        send_checksum_error++;

                    for (unsigned int i = 0; i < data_tosend_1.size(); i++)
                    {
                        double tmp = data_tosend_1[i] - trama[data_received.size() + 3 + i];

                        if (tmp < 0)
                            tmp = -1 * tmp;
                        if (tmp > 1e-9)
                        {
                            send_checksum_error++;
                        }
                    }

                    if (send_checksum_error == 0 || send_error > max_send_error)
                    { // el dato anterior esta correcto o ya no tiene valides
                        data_tosend_1 = data_tosend;
                        data_tosend_type_1 = data_tosend_type;
                        send_error = 0;
                        send_checksum_error = 0;
                    }
                    else
                    {
                        send_error++;
                    }
                }
                else
                {
                    receive_error++; // error por checksum

                    send_error++;
                    ;
                    send_checksum_error = -1;
                }
            }
            else
            {
                receive_error++;

                send_error++;
                ;
                send_checksum_error = -2; // error por tama�o de la trama
            }
            return 0;
        }

        /// it is generated the trama to send to server (first the data to be send, then the data to be received) taking account the last trama received
        void send_data()
        {
            trama[0] = (double)send_checksum_error;
            trama[1] = (double)data_tosend_type_1;
            trama[2] = (double)data_received_type;
            std::copy(data_tosend_1.begin(), data_tosend_1.end(), trama.begin() + 3);
            std::copy(data_received.begin(), data_received.end(), trama.begin() + data_tosend_1.size() + 3);

            // trama[data_received.size()+data_tosend_1.size()+3]=111;
#ifdef DEBUG
            std::cout << "datos para enviar \n";
            for (int i = 0; i < trama.size() - 1; i++)
                std::cout << (double)trama[i] << ",";
#endif

            // prot.maencode<double>(buffer_tosend,trama,tipodato);
            int i = maencode(&buffer_tosend[0], &trama[0], &tipodato[0], trama.size() - 1, processortype);

            // add checksum
            checksum_1 = crc_msb(&buffer_tosend[0], buffer_tosend.size() - 4, GEN_ANSI_MSB);
            trama[trama.size() - 1] = checksum_1;
            ds_uint32_t temp0;
            temp0.int32_r = (unsigned int)checksum_1;
            if (processortype == BIGENDIAN)
            {
                buffer_tosend[buffer_tosend.size() - 1] = temp0.uint32_r.byte0;
                buffer_tosend[buffer_tosend.size() - 2] = temp0.uint32_r.byte1;
                buffer_tosend[buffer_tosend.size() - 3] = temp0.uint32_r.byte2;
                buffer_tosend[buffer_tosend.size() - 4] = temp0.uint32_r.byte3;
            }
            else
            {
                buffer_tosend[buffer_tosend.size() - 1] = temp0.uint32_r.byte3;
                buffer_tosend[buffer_tosend.size() - 2] = temp0.uint32_r.byte2;
                buffer_tosend[buffer_tosend.size() - 3] = temp0.uint32_r.byte1;
                buffer_tosend[buffer_tosend.size() - 4] = temp0.uint32_r.byte0;
            }
            // prot.maencode<double>(buffer_tosend,trama,tipodato);
#ifdef DEBUG
            std::cout << trama[trama.size() - 1] << ":" << trama.size() << "\n";
#endif
        }
        std::vector<double> data_tosend_1;
        std::vector<double> data_received_1;
        std::vector<double> data_tosend;
        std::vector<double> data_received;
        unsigned short data_tosend_type;
        unsigned short data_tosend_type_1;
        unsigned short data_received_type;
        unsigned short data_received_type_1;
        unsigned int bytes_udp;
        unsigned int checksum;
        unsigned int checksum_1;
        int receive_error;
        int send_error;
        int max_send_error;
        int processortype;
        int send_checksum_error;

        std::vector<double> trama;
        std::vector<unsigned char> buffer_tosend;
        std::vector<unsigned char> buffer_received;
        std::vector<int> tipodato;
        microautobox::udp_socket _udp_socket;
    };

} /* namespace microautobox */
#endif /* MAUDP_H_ */