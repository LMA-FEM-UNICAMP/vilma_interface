/*
 * protocolo.h
 * library to encode and decode an array of double in bynary
 * with the possibility to encoding this variable in
 * @ref datos {manull,MA_BOOLEAN,MA_INT8,MA_UINT8,MA_INT16,MA_UINT16,MA_INT32,MA_UINT32,MA_FLOAT,MA_DOUBLE}

 *
 * it is based of the example of Dspace  microautobox II to communicate by udp
 *
 * FILE:
 *   ds867c_eth_encode32_sfcn.c
 *
 * DESCRIPTION:
 * Encoding of data in various input format to 32bit WORD format
 * it should work both in Simulink as well as in RT Application with dSPACE HW
 * LaszloJ
 * FILE:
 *   ds867c_eth_decode32_sfcn.c
 *
 * DESCRIPTION:
 * decodes the 32bit WORD-coded data into selected data types
 *
 * Used and changed by LaszloJ
 *
 * @author olmer Garcia olmerg@gmail.com
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

#ifndef PROTOCOLOC_H_
#define PROTOCOLOC_H_
#ifdef __cplusplus
extern "C"
{
#endif

// #include <inttypes.h>
#define GEN_ANSI_MSB 0x04C11DB7
#define MA_BOOLEAN 1
#define MA_INT8 2
#define MA_UINT8 3
#define MA_INT16 4
#define MA_UINT16 5
#define MA_INT32 6
#define MA_UINT32 7
#define MA_FLOAT 8
#define MA_DOUBLE 9

#define BIGENDIAN 1
#define LITLEENDIAN 2
#define MAX_LENGTH 65536

	/**
	 * types of variables available in the microautobox
	 * Datatype Codes:
	BOOLEAN:  1   //   INT8      :  2    //   UINT8    :  3
	INT16       :  4   //   UINT16 :  5    //   INT32     :  6
	UINT32    :  7   //   FLOAT   :  8    //   DOUBLE:  9
	 */

	/*template <typename T> int typeofi(T&)  {return manull;}
	template<> int typeofi(bool&) { return MA_BOOLEAN; }
	template<> int typeofi(char&) { return MA_INT8; }
	template<> int typeofi(unsigned char&) { return MA_UINT8; }
	template<> int typeofi(short &) { return MA_INT16; }
	template<> int typeofi(unsigned short&) { return MA_UINT16; }
	template<> int typeofi(int&) { return MA_INT32; }
	template<> int typeofi(unsigned int&) { return MA_UINT32; }
	template<> int typeofi(float&) { return MA_FLOAT; }
	template<> int typeofi(double&) { return MA_DOUBLE; }*/

	typedef struct
	{
		unsigned char byte7;
		unsigned char byte6;
		unsigned char byte5;
		unsigned char byte4;
		unsigned char byte3;
		unsigned char byte2;
		unsigned char byte1;
		unsigned char byte0;
	} uint64_by_uint8_t;

	typedef struct
	{
		unsigned char byte3;
		unsigned char byte2;
		unsigned char byte1;
		unsigned char byte0;
	} uint32_by_uint8_t;

	/*===============================*/

	typedef struct
	{
		unsigned char byte1;
		unsigned char byte0;
	} uint16_by_uint8_t;

	/*===============================*/

	typedef union
	{
		uint32_by_uint8_t uint32_r;
		float float32_r;
	} float32_t;

	/*===============================*/

	typedef union
	{
		uint64_by_uint8_t uint64_r;
		double float64_r;
	} float64_t;

	/*===============================*/

	typedef union
	{
		uint32_by_uint8_t uint32_r;
		unsigned int int32_r;
	} ds_uint32_t;

	/*===============================*/

	typedef union
	{
		uint16_by_uint8_t uint16_r;
		unsigned short int16_r;
	} ds_uint16_t;

	/***
	 * @brief function to convert an array of double to unsigned char
	 * @param the vector with the data encoded in unsigned char to be transmited in bynary form
	 * @param data is vector of T with the data to encode in unsigned char
	 * @param tipe @ref datos {manull,MA_BOOLEAN,MA_INT8,MA_UINT8,MA_INT16,MA_UINT16,MA_INT32,MA_UINT32,MA_FLOAT,MA_DOUBLE}
	 * @return 0 is ok, otherwise error
	 */

	int maencode(unsigned char *buff, double *trama, int *tipe, int length_trama, int PROCESSORTYPE)
	{

		ds_uint16_t temp;
		ds_uint32_t temp0;
		float32_t temp1;
		float64_t temp2;
		int buff_counter = 0;
		int i = 0;
		if (PROCESSORTYPE == BIGENDIAN) /* big-endian*/
		{
			for (i = 0; i < length_trama; i++)
			{
				switch (tipe[i])
				{
				case MA_BOOLEAN:
					if (trama[i] == 0)
						buff[buff_counter++] = 0;
					else
						buff[buff_counter++] = 1;
					break;
				case MA_INT8:
				case MA_UINT8:
					buff[buff_counter++] = (unsigned char)trama[i];
					break;
				case MA_INT16:
				case MA_UINT16:
					temp.int16_r = (unsigned short)trama[i];
					buff[buff_counter++] = temp.uint16_r.byte1;
					buff[buff_counter++] = temp.uint16_r.byte0;
					break;
				case MA_INT32:
				case MA_UINT32:
					temp0.int32_r = (unsigned int)trama[i];
					buff[buff_counter++] = temp0.uint32_r.byte3;
					buff[buff_counter++] = temp0.uint32_r.byte2;
					buff[buff_counter++] = temp0.uint32_r.byte1;
					buff[buff_counter++] = temp0.uint32_r.byte0;
					break;
				case MA_FLOAT:
					temp1.float32_r = (float)trama[i];
					buff[buff_counter++] = temp1.uint32_r.byte3;
					buff[buff_counter++] = temp1.uint32_r.byte2;
					buff[buff_counter++] = temp1.uint32_r.byte1;
					buff[buff_counter++] = temp1.uint32_r.byte0;
					break;
				case MA_DOUBLE:
					temp2.float64_r = (double)trama[i];
					buff[buff_counter++] = temp2.uint64_r.byte7;
					buff[buff_counter++] = temp2.uint64_r.byte6;
					buff[buff_counter++] = temp2.uint64_r.byte5;
					buff[buff_counter++] = temp2.uint64_r.byte4;
					buff[buff_counter++] = temp2.uint64_r.byte3;
					buff[buff_counter++] = temp2.uint64_r.byte2;
					buff[buff_counter++] = temp2.uint64_r.byte1;
					buff[buff_counter++] = temp2.uint64_r.byte0;
					break;
				}
			}
		}
		else /* litle-endian*/
		{
			for (i = 0; i < length_trama; i++)
			{
				switch (tipe[i])
				{
				case (int)MA_BOOLEAN:
					if (trama[i] == 0)
						buff[buff_counter++] = 0;
					else
						buff[buff_counter++] = 1;
					break;
				case MA_INT8:
				case MA_UINT8:
					buff[buff_counter++] = (unsigned char)trama[i];
					break;
				case MA_INT16:
				case MA_UINT16:
					temp.int16_r = (unsigned short)trama[i];
					buff[buff_counter++] = temp.uint16_r.byte0;
					buff[buff_counter++] = temp.uint16_r.byte1;
					break;
				case MA_INT32:
				case MA_UINT32:
					temp0.int32_r = (unsigned int)trama[i];
					buff[buff_counter++] = temp0.uint32_r.byte0;
					buff[buff_counter++] = temp0.uint32_r.byte1;
					buff[buff_counter++] = temp0.uint32_r.byte2;
					buff[buff_counter++] = temp0.uint32_r.byte3;
					break;
				case MA_FLOAT:
					temp1.float32_r = (float)trama[i];
					buff[buff_counter++] = temp1.uint32_r.byte0;
					buff[buff_counter++] = temp1.uint32_r.byte1;
					buff[buff_counter++] = temp1.uint32_r.byte2;
					buff[buff_counter++] = temp1.uint32_r.byte3;
					break;
				case MA_DOUBLE:
					temp2.float64_r = (double)trama[i];
					buff[buff_counter++] = temp2.uint64_r.byte0;
					buff[buff_counter++] = temp2.uint64_r.byte1;
					buff[buff_counter++] = temp2.uint64_r.byte2;
					buff[buff_counter++] = temp2.uint64_r.byte3;
					buff[buff_counter++] = temp2.uint64_r.byte4;
					buff[buff_counter++] = temp2.uint64_r.byte5;
					buff[buff_counter++] = temp2.uint64_r.byte6;
					buff[buff_counter++] = temp2.uint64_r.byte7;
					break;
				}
			}
		}

		return 0;
	};
	/***
	 *
	 * @param data
	 * @param tipe an array of the type of data sent @ref datos {manull,MA_BOOLEAN,MA_INT8,MA_UINT8,MA_INT16,MA_UINT16,MA_INT32,MA_UINT32,MA_FLOAT,MA_DOUBLE}
	 * @return  0 is ok
	 */

	int madecode(double *trama, unsigned char *data, int *tipe, int length_tipe, int PROCESSORTYPE)
	{
		ds_uint16_t temp;
		ds_uint32_t temp0;
		float32_t temp1;
		float64_t temp2;
		unsigned int offset = 0;
		int i = 0;
		if (PROCESSORTYPE == BIGENDIAN) /* big-endian*/
		{
			for (i = 0; i < length_tipe; i++)
			{
				switch (tipe[i])
				{
				case MA_INT8:
				case MA_BOOLEAN:
					trama[i] = (int)data[offset];
					++offset;
					break;
				case MA_UINT8:
					trama[i] = (unsigned int)data[offset];
					++offset;
					break;
				case MA_INT16:
					temp.uint16_r.byte1 = data[offset];
					temp.uint16_r.byte0 = data[++offset];
					++offset;
					trama[i] = (short)temp.int16_r;
					break;
				case MA_UINT16:
					temp.uint16_r.byte1 = data[offset];
					temp.uint16_r.byte0 = data[++offset];
					++offset;
					trama[i] = (unsigned short)temp.int16_r;
					break;
				case MA_INT32:
					temp0.uint32_r.byte3 = data[offset];
					temp0.uint32_r.byte2 = data[++offset];
					temp0.uint32_r.byte1 = data[++offset];
					temp0.uint32_r.byte0 = data[++offset];
					++offset;
					trama[i] = (int)temp0.int32_r;
					break;
				case MA_UINT32:
					temp0.uint32_r.byte3 = data[offset];
					temp0.uint32_r.byte2 = data[++offset];
					temp0.uint32_r.byte1 = data[++offset];
					temp0.uint32_r.byte0 = data[++offset];
					++offset;
					trama[i] = (unsigned int)temp0.int32_r;
					break;
				case MA_FLOAT:
					temp1.uint32_r.byte3 = data[offset];
					temp1.uint32_r.byte2 = data[++offset];
					temp1.uint32_r.byte1 = data[++offset];
					temp1.uint32_r.byte0 = data[++offset];
					++offset;
					trama[i] = (double)temp1.float32_r;
					break;
				case MA_DOUBLE:
					temp2.uint64_r.byte7 = data[offset];
					temp2.uint64_r.byte6 = data[++offset];
					temp2.uint64_r.byte5 = data[++offset];
					temp2.uint64_r.byte4 = data[++offset];
					temp2.uint64_r.byte3 = data[++offset];
					temp2.uint64_r.byte2 = data[++offset];
					temp2.uint64_r.byte1 = data[++offset];
					temp2.uint64_r.byte0 = data[++offset];
					++offset;
					trama[i] = (double)temp2.float64_r;
					break;
				}
			}
		}
		else /* litle-endian*/
		{

			for (i = 0; i < length_tipe; i++)
			{
				switch (tipe[i])
				{
				case MA_INT8:
				case MA_BOOLEAN:
					trama[i] = (int)data[offset];
					++offset;
					break;
				case MA_UINT8:
					trama[i] = (unsigned int)data[offset];
					++offset;
					break;
				case MA_INT16:
					temp.uint16_r.byte0 = data[offset];
					temp.uint16_r.byte1 = data[++offset];
					++offset;
					trama[i] = (short)temp.int16_r;
					break;
				case MA_UINT16:
					temp.uint16_r.byte0 = data[offset];
					temp.uint16_r.byte1 = data[++offset];
					++offset;
					trama[i] = (unsigned short)temp.int16_r;
					break;
				case MA_INT32:
					temp0.uint32_r.byte0 = data[offset];
					temp0.uint32_r.byte1 = data[++offset];
					temp0.uint32_r.byte2 = data[++offset];
					temp0.uint32_r.byte3 = data[++offset];
					++offset;
					trama[i] = (int)temp0.int32_r;
					break;
				case MA_UINT32:
					temp0.uint32_r.byte0 = data[offset];
					temp0.uint32_r.byte1 = data[++offset];
					temp0.uint32_r.byte2 = data[++offset];
					temp0.uint32_r.byte3 = data[++offset];

					++offset;
					trama[i] = (unsigned int)temp0.int32_r;
					break;
				case MA_FLOAT:
					temp1.uint32_r.byte0 = data[offset];
					temp1.uint32_r.byte1 = data[++offset];
					temp1.uint32_r.byte2 = data[++offset];
					temp1.uint32_r.byte3 = data[++offset];
					++offset;
					trama[i] = (double)temp1.float32_r;
					break;
				case MA_DOUBLE:
					temp2.uint64_r.byte0 = data[offset];
					temp2.uint64_r.byte1 = data[++offset];
					temp2.uint64_r.byte2 = data[++offset];
					temp2.uint64_r.byte3 = data[++offset];
					temp2.uint64_r.byte4 = data[++offset];
					temp2.uint64_r.byte5 = data[++offset];
					temp2.uint64_r.byte6 = data[++offset];
					temp2.uint64_r.byte7 = data[++offset];
					++offset;
					trama[i] = (double)temp2.float64_r;
					break;
				}
			}
		}

		return 0;
	};
	/***
	 * @param data in unsigned int to be returned
	 * @param data in unsigned char
	 * @return  0 is ok
	 */
	int maencodetouint32(unsigned int *buff, unsigned char *data, int length_bytes, int PROCESSORTYPE)
	{
		ds_uint32_t temp0;
		int size = 0;
		int i = 0;
		int _offset = 0;
		if ((length_bytes % 4) == 0)
		{
			size = length_bytes / 4;
		}
		else
		{
			size = length_bytes / 4 + 1;
		}

		for (i = 0; i < size; i++)
		{
			if (PROCESSORTYPE == BIGENDIAN)
			{
				temp0.uint32_r.byte3 = data[_offset];
				temp0.uint32_r.byte2 = data[++_offset];
				temp0.uint32_r.byte1 = data[++_offset];
				temp0.uint32_r.byte0 = data[++_offset];
				++_offset;
				buff[i] = (unsigned int)temp0.int32_r;
			}
			else
			{ /* litle-endian*/
				temp0.uint32_r.byte0 = data[_offset];
				temp0.uint32_r.byte1 = data[++_offset];
				temp0.uint32_r.byte2 = data[++_offset];
				temp0.uint32_r.byte3 = data[++_offset];

				++_offset;
				buff[i] = (unsigned int)temp0.int32_r;
			}
		}

		return 0;
	};
	/***
	 * @param buff in unsigned char to be returned
	 * @param data in unsigned int received buffer
	 * @return  0 is ok
	 */
	int maencodetobyte(unsigned char *buff, unsigned int *data, int length_bytes, int PROCESSORTYPE)
	{
		ds_uint32_t temp0;
		int buff_counter = 0;
		int i = 0;
		int size = 0;
		if ((length_bytes % 4) == 0)
		{
			size = length_bytes / 4;
		}
		else
		{
			size = length_bytes / 4 + 1;
		}
		for (i = 0; i < size; i++)
		{
			if (PROCESSORTYPE == BIGENDIAN)
			{
				temp0.int32_r = (unsigned int)data[i];
				buff[buff_counter++] = temp0.uint32_r.byte3;
				buff[buff_counter++] = temp0.uint32_r.byte2;
				buff[buff_counter++] = temp0.uint32_r.byte1;
				buff[buff_counter++] = temp0.uint32_r.byte0;
			}
			else
			{ /* litle-endian*/
				temp0.int32_r = (unsigned int)data[i];
				buff[buff_counter++] = temp0.uint32_r.byte0;
				buff[buff_counter++] = temp0.uint32_r.byte1;
				buff[buff_counter++] = temp0.uint32_r.byte2;
				buff[buff_counter++] = temp0.uint32_r.byte3;
			}
		}

		return 0;
	};

	unsigned int crc_msb(const unsigned char *data, int size, unsigned int generator)
	{
		static const unsigned char BITS = 8 * sizeof(unsigned int);
		unsigned int r = 0;

		int i = 0, j = 0;
		for (i = 0; i < size; i++)
		{
			r ^= data[i] << (BITS - 8);
			for (j = 0; j < 8; j++)
				r = (r & (1 << (BITS - 1)) ? (r << 1) ^ generator : r << 1);
		}

		return r;
	};
	// Direct computation, lsb-first
	unsigned int crc_lsb(const unsigned char *data, int size, unsigned int generator)
	{
		unsigned int r = 0;

		int i = 0, j = 0;
		for (i = 0; i < size; i++)
		{
			r ^= data[i];
			for (j = 0; j < 8; j++)
				r = (r & 0x01 ? (r >> 1) ^ generator : r >> 1);
		}

		return r;
	};

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOLOC_H_ */
