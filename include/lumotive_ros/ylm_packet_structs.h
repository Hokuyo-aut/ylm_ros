// Copyright (C) 2021, 2022 Lumotive
// Copyright 2023 HOKUYO AUTOMATIC CO.,LTD.

#ifndef YLM_PACKET_STRUCTS_H
#define YLM_PACKET_STRUCTS_H

#include <boost/endian/arithmetic.hpp>
#define BOOST_ENDIAN_FORCE_POD_TYPES

struct TypeDReturn_t
{
	boost::endian::big_uint16_t intensity;
	boost::endian::big_uint16_t range;
	boost::endian::big_uint16_t background;
	boost::endian::big_uint16_t snr;
	boost::endian::big_uint8_t extra_annotation;
	boost::endian::big_uint8_t return_flags;

	bool isSet_rangePresentAndValid() const {return return_flags & rangePresentAndValid;}
	bool isSet_intensityPresentAndValid() const {return return_flags & intensityPresentAndValid;}
	bool isSet_backgroundPresentAndValid() const {return return_flags & backgroundPresentAndValid;}
	bool isSet_snrPresentAndValid() const {return return_flags & snrPresentAndValid;}
	bool isSet_extraTypeNNCount() const {return return_flags & extraTypeNNCount;}

	// For compatibility with Type 3 Returns, and to encode semantic that zero range means
	// entire return should be ignored
	bool isSet_returnPresentAndValid() const {return isSet_rangePresentAndValid() && (range != 0);}

	enum ReturnFlags
	{
		rangePresentAndValid = 1,
		intensityPresentAndValid = 2,
		backgroundPresentAndValid = 4,
		snrPresentAndValid = 8,
		extraTypeNNCount = 16
	};

	static const size_t read_len;
};
const size_t TypeDReturn_t::read_len = sizeof(TypeDReturn_t);
static_assert(std::is_pod<TypeDReturn_t>::value == true, "Packet struct types must compile as Plain-Old-Data");

struct DataDesc_t
{
	boost::endian::big_uint8_t length;
	boost::endian::big_uint8_t encoding;
	boost::endian::big_uint16_t type;

	enum LengthValues
	{
		Fixed1Byte = 1,
		Fixed2Byte = 2,
		Fixed4Byte = 4,
		Fixed8Byte = 8,
		Fixed16Byte = 16
	};


	enum EncodingValues
	{
		TypeSpecific = 0,
		UnsignedInteger = 1,
		TwosComplementInteger = 2,
		IEEE754 = 3,
		TypeSpecificPair = 128,
		UnsignedIntegerPair = 129,
		TwosComplementIntegerPair = 130,
		IEEE754Pair = 131
	};

	enum TypeValues
	{
		Reserved = 0,
		Range = 1,
		Signal = 2,
		Background = 3,
		SNR = 4,
		Intensity = 5,
		CoordinateTheta = 16385,
		CoordinatePhi = 16386,
		CoordinateU = 16387,
		CoordinateV = 16388,
		RawSensorData = 16389,
  		RawSensorDataPacked = 16340,
		AngleMapping = 32769
	};

	// When parameter is fixed length, length in bytes
	// -1 otherwise
	int get_parameter_len() const
	{
		switch (length)
		{
		case DataDesc_t::LengthValues::Fixed1Byte:
			return 1;
			break;
		case DataDesc_t::LengthValues::Fixed2Byte:
			return 2;
			break;
		case DataDesc_t::LengthValues::Fixed4Byte:
			return 4;
			break;
		case DataDesc_t::LengthValues::Fixed8Byte:
			return 8;
			break;
		case DataDesc_t::LengthValues::Fixed16Byte:
			return 16;
			break;
		default:
			return -1;
			break;
		}
		return -1;
	}

	static const size_t read_len;
};
const size_t DataDesc_t::read_len = sizeof(DataDesc_t);
static_assert(std::is_pod<DataDesc_t>::value == true, "Packet struct types must compile as Plain-Old-Data");

struct TypeCParameter_t
{
	uint8_t raw_data[1];	//Placeholder

	static const size_t read_len;
};
const size_t TypeCParameter_t::read_len = sizeof(TypeCParameter_t);
static_assert(std::is_pod<TypeCParameter_t>::value == true, "Packet struct types must compile as Plain-Old-Data");


// Specialized version of TypeCParameter_t for the only-known type: coordinateMap32ASTheta32ASPhi
struct TypeCParameter_CM32T32P_t {
    boost::endian::big_int32_t theta;
    boost::endian::big_int32_t phi;

	static const size_t read_len;
};
const size_t TypeCParameter_CM32T32P_t::read_len = sizeof(TypeCParameter_CM32T32P_t);
static_assert(std::is_pod<TypeCParameter_CM32T32P_t>::value == true, "Packet struct types must compile as Plain-Old-Data");


struct TypeCHeader_t
{
	boost::endian::big_uint16_t image_end_u;
	boost::endian::big_uint16_t image_end_v;
	boost::endian::big_uint16_t payload_start_u;
	boost::endian::big_uint16_t payload_start_v;
	uint8_t  parameter_type;
	boost::endian::big_uint8_t reserved0;
	boost::endian::big_uint16_t reserved1;
	boost::endian::big_uint32_t reserved2;

	enum class TypeCParameterTypeCode {
		// Whenever in doubt, Big Endian
		none = 1,						  // Null/Unused
		coordinateMap32ASTheta32ASPhi = 2 // int32_t Theta in Arc Seconds, int32_t Phy in Arc Seconds
	};

	static const size_t read_len;

	TypeCParameter_t* get_nextSection_TypeCParameters() const {
		return (TypeCParameter_t *) ((uint8_t *)this+read_len);
	}

	TypeCParameter_CM32T32P_t* get_nextSection_TypeCParameters_CM32T32P() const {
		if (parameter_type == (int)TypeCParameterTypeCode::coordinateMap32ASTheta32ASPhi)
			return (TypeCParameter_CM32T32P_t *) ((uint8_t *)this+read_len);
		else
			return nullptr;
	}
};
const size_t TypeCHeader_t::read_len = sizeof(TypeCHeader_t);
static_assert(std::is_pod<TypeCHeader_t>::value == true, "Packet struct types must compile as Plain-Old-Data");


struct TypeDHeader_t
{
	boost::endian::big_uint8_t timestamp[10];
	boost::endian::big_uint8_t tscale__ao_seq_flags;
	boost::endian::big_uint32_t ao_ls_start;
	boost::endian::big_uint32_t ao_ls_end;
	boost::endian::big_uint32_t ao_cs_start;
	boost::endian::big_uint32_t ao_cs_end;
	boost::endian::big_uint16_t complete_size_steer_dim;
	boost::endian::big_uint16_t complete_size_stare_dim;
	boost::endian::big_uint16_t payload_steerorder_offset;
	boost::endian::big_uint16_t payload_stareorder_offset;
	boost::endian::big_uint16_t payload_start_v;
	boost::endian::big_uint16_t payload_step_v;
	boost::endian::big_uint16_t payload_start_u;
	boost::endian::big_uint16_t payload_step_u;
	boost::endian::big_uint16_t user_tag;

	uint8_t extract_tscale() const {return (tscale__ao_seq_flags & 0xF0) >> 4;}
	uint8_t extract_ao_seq_flags() const {return tscale__ao_seq_flags & 0x0F;}

	bool isSet_lastSceneBeginSequenceValid() const {return extract_ao_seq_flags() & lastSceneBeginSequenceValid;}
	bool isSet_lastSceneEndSequenceValid() const {return extract_ao_seq_flags() & lastSceneEndSequenceValid;}
	bool isSet_currentSceneBeginSequenceValid() const {return extract_ao_seq_flags() & currentSceneBeginSequenceValid;}
	bool isSet_currentSceneEndSequenceValid() const {return extract_ao_seq_flags() & currentSceneEndSequenceValid;}

	enum AOSeqFlags
	{
		lastSceneBeginSequenceValid = 1,
		lastSceneEndSequenceValid = 2,
		currentSceneBeginSequenceValid = 4,
		currentSceneEndSequenceValid = 8
	};

	static const size_t read_len;
	static const size_t payload_count_TypeDReturns = 64;

	TypeDReturn_t* get_payloads_TypeDReturns() const {
		return (TypeDReturn_t *) ((uint8_t *)this+read_len);
	}

	TypeDReturn_t* get_payload_TypeDReturn(size_t index) const {
		if(index < payload_count_TypeDReturns)
			return &(get_payloads_TypeDReturns()[index]);
		return nullptr;
	}
};
const size_t TypeDHeader_t::read_len = sizeof(TypeDHeader_t);
static_assert(std::is_pod<TypeDHeader_t>::value == true, "Packet struct types must compile as Plain-Old-Data");

struct GlobalHeader_t
{
	boost::endian::big_uint8_t magic[4];
	boost::endian::big_uint8_t version_type;
	boost::endian::big_uint32_t device_version;
	boost::endian::big_uint32_t seq;
	boost::endian::big_uint32_t device_id;
	boost::endian::big_uint32_t reserved;

	uint8_t extract_version() const {return (version_type & 0xF0) >> 4;}
	uint8_t extract_type() const {return version_type & 0x0F;}

	static const size_t read_len;

	void* get_nextHeader() const {
		return ((uint8_t *)this+read_len);
	}

	TypeDHeader_t* get_nextHeader_TypeD() const {
		if(extract_type() == 0xD)
			return (TypeDHeader_t *) ((uint8_t *)this+read_len);
		return nullptr;
	}

	TypeCHeader_t* get_nextHeader_TypeC() const {
		if(extract_type() == 0xC)
			return (TypeCHeader_t *) ((uint8_t *)this+read_len);
		return nullptr;
	}

};
const size_t GlobalHeader_t::read_len = sizeof(GlobalHeader_t);
static_assert(std::is_pod<GlobalHeader_t>::value == true, "Packet struct types must compile as Plain-Old-Data");

struct TCPFramingHeader_t
{
	boost::endian::big_uint32_t len;
	boost::endian::big_uint32_t flags;
	boost::endian::big_uint32_t echoVal;
	boost::endian::big_uint32_t reserved;

	static const size_t read_len;

};
const size_t TCPFramingHeader_t::read_len = sizeof(TCPFramingHeader_t);
static_assert(std::is_pod<TCPFramingHeader_t>::value == true, "Packet struct types must compile as Plain-Old-Data");


// Utilities for manipulating PTP Timestamps
struct PTPTimestamp_t
{
	public:
		PTPTimestamp_t (uint8_t *data) {
			// Pad out 80 (48||32) bit timestamp to 96 (64||32) bits, extract big endian values
			uint8_t localData[12] = {0};
			memcpy(localData + 2, data, 10);
			boost::endian::big_int64_t secField = *((boost::endian::big_int64_t *) localData);
			boost::endian::big_int32_t nsecField = *((boost::endian::big_int32_t *) (localData + 8));

			m_sec = secField;
			m_nsec = nsecField;
		}

		struct timespec asTimespec() const {
			struct timespec retVal;
			retVal.tv_sec = m_sec;
			retVal.tv_nsec = m_nsec;

			return retVal;
		}

		struct timeval asTimeval() const {
			struct timeval retVal;
			retVal.tv_sec = m_sec;
			retVal.tv_usec = m_nsec / 1000;

			return retVal;
		}

	private:
		uint64_t m_sec;
		uint32_t m_nsec;

		time_t m_tv_sec;
		long int m_tv_nsec;
		long int m_tv_usec;
};

#endif