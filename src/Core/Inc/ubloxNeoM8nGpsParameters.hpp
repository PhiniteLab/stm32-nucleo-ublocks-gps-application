#ifndef __UBLOX_NEO_M8N_GPS_PARAMETERS_HPP__
#define __UBLOX_NEO_M8N_GPS_PARAMETERS_HPP__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "string.h"

#ifdef __cplusplus
}
#endif

/*
 * UBX Protocol :
 * -> [SYNC CHAR 1][SYNC CHAR 2][CLASS][ID][LENGTH][Payload][Checksum A][Checksum B]
 * -> SYNC CHAR 1 : 0xB5
 * -> SYNC CHAR 2 : 0x62
 * -> CLASS :	1-byte Message Class
 * -> ID : 	1-byte Message ID
 * -> Length:	2-byte length of UBX message payload (little endian)
 * -> Payload : field contains a variable number of bytes
 * -> Checksum A: 8 bit unsigned integer
 * -> Checksum B: 8 bit unsigned integer
 */

/*** !! NOTES !!  : for position values : set UBX_NAV_PVT message before
 ****     			and check your UART settings like baudrate, stop bits etc.*/

///////////////////////////////////////////////////////////
// UBX Protocol SYNC Macro
#define HEADER_INIT								0xB5
#define HEADER_SECOND							0x62

// UBX Protocol SYNC Macro
///////////////////////////////////////////////////////////

///////////////////////////////////////////////////
// State Machine Macro

#define HEADER_CAPTURE_FIRST           	    	0x01
#define HEADER_CAPTURE_SECOND                 	0x02
#define CLASS_CAPTURE             				0x03
#define ID_CAPTURE            					0x04
#define LENGTH_CAPTURE                			0x05
#define PAYLOAD_CAPTURE                       	0x06
#define CHECKSUM_A_CAPTURE                      0x07
#define CHECKSUM_B_CAPTURE                		0x08

// State Machine Macro
///////////////////////////////////////////////////

#define CAPTURE_PACKAGE_SIZE					500
#define PAYLOAD_SIZE							100

#define NAV_CLASS								0x01
#define PVT_MSG_ID								0x07

class ublox_neo_m8n_gps_parameters {

private:

	UART_HandleTypeDef *uartType; // gps uart connection type

	uint8_t recevied_data_dma_u8[CAPTURE_PACKAGE_SIZE]; // coming data array from gps

	////////////////////////////////////////
	// State Machine Parameters

	uint8_t capture_status_u8; // State flag parameter

	uint8_t data_array_u8[CAPTURE_PACKAGE_SIZE]; // byte array for state machine algorithm

	uint16_t data_counter_u16; // counter index for data array

	uint8_t package_class_u8;   // Class of captured package
	uint8_t package_id_u8;      // ID of capture package
	uint8_t payload_size_u8;    // Bytes size of capture payload

	uint8_t payload_array_u8[PAYLOAD_SIZE]; // captured payload

	uint8_t CK_A_u8; // captured check-sum A value
	uint8_t CK_B_u8; // captured check-sum B value

	uint8_t calculated_CK_A_u8; // calculated check-sum A value
	uint8_t calculated_CK_B_u8; // calculated check-sum B value

	// State Machine Parameters
	////////////////////////////////////////
	typedef struct ubx_nav_pvt_package {

		//////////////////////////////
		// Position parameters

		float latitude;
		float longitude;
		float height_above_ellipsoid;
		float height_above_mean_sea;

		// Position parameters
		//////////////////////////////

		/////////////////////////////
		// Time parameters

		uint16_t year_u16;
		uint8_t month_u8;
		uint8_t day_u8;
		uint8_t hour_u8;
		uint8_t min_u8;
		uint8_t second_u8;

		// Time parameters
		////////////////////////////

		///////////////////////////
		// GPS fix type

		uint8_t fix_type_u8;

		// GPS fix type
		///////////////////////////

	} ubx_nav_pvt_package_s;

public:

	ubx_nav_pvt_package_s nav_pvt_pavkage_st;

	ublox_neo_m8n_gps_parameters(UART_HandleTypeDef *uart) {

		this->uartType = uart; // assign uart
		memset(this->recevied_data_dma_u8, 0, CAPTURE_PACKAGE_SIZE); // clear received msg array

		/////////////////////////////////////////////////////
		// UBX-NAV-PVT msg initial value assign

		this->nav_pvt_pavkage_st.latitude = 0;
		this->nav_pvt_pavkage_st.longitude = 0;
		this->nav_pvt_pavkage_st.height_above_ellipsoid = 0;
		this->nav_pvt_pavkage_st.height_above_mean_sea = 0;

		this->nav_pvt_pavkage_st.year_u16 = 0;
		this->nav_pvt_pavkage_st.month_u8 = 0;
		this->nav_pvt_pavkage_st.day_u8 = 0;
		this->nav_pvt_pavkage_st.hour_u8 = 0;
		this->nav_pvt_pavkage_st.min_u8 = 0;
		this->nav_pvt_pavkage_st.second_u8 = 0;

		this->nav_pvt_pavkage_st.fix_type_u8 = 0;

		// UBX-NAV-PVT msg initial value assign
		/////////////////////////////////////////////////////

		/////////////////////////////////////////////////////
		// Assign initial value state machine parameters

		this->capture_status_u8 = HEADER_CAPTURE_FIRST;

		memset(this->data_array_u8, 0, CAPTURE_PACKAGE_SIZE);
		this->data_counter_u16 = 0;

		this->package_class_u8 = 0;
		this->package_id_u8 = 0;
		this->payload_size_u8 = 0;

		memset(this->payload_array_u8, 0, PAYLOAD_SIZE);

		this->CK_A_u8 = 0;
		this->CK_B_u8 = 0;

		this->calculated_CK_A_u8 = 0;
		this->calculated_CK_B_u8 = 0;

		// Assign initial value state machine parameters
		/////////////////////////////////////////////////////

	}

	///////////////////////////////////////////////////////////
	// Parse gps message with State Machine Algorithm

	void get_gps_message() {

		for (int i = 0; i < CAPTURE_PACKAGE_SIZE; i++) {

			switch (this->capture_status_u8) {

			case HEADER_CAPTURE_FIRST: {

				if (this->recevied_data_dma_u8[i] == HEADER_INIT) {

					this->data_array_u8[this->data_counter_u16] =
							this->recevied_data_dma_u8[i];

					this->data_counter_u16++;
					this->capture_status_u8 = HEADER_CAPTURE_SECOND;
				}

				else {

					this->capture_status_u8 = HEADER_CAPTURE_FIRST;
					this->data_counter_u16 = 0;

				}

				break;
			}

			case HEADER_CAPTURE_SECOND: {

				if (this->recevied_data_dma_u8[i] == HEADER_SECOND) {

					this->data_array_u8[this->data_counter_u16] =
							this->recevied_data_dma_u8[i];

					this->data_counter_u16++;
					this->capture_status_u8 = CLASS_CAPTURE;
				}

				else {

					this->capture_status_u8 = HEADER_CAPTURE_FIRST;
					this->data_counter_u16 = 0;

				}

				break;
			}

			case CLASS_CAPTURE: {

				this->data_array_u8[this->data_counter_u16] =
						this->recevied_data_dma_u8[i];

				this->package_class_u8 =
						this->data_array_u8[this->data_counter_u16];

				this->data_counter_u16++;
				this->capture_status_u8 = ID_CAPTURE;

				break;
			}

			case ID_CAPTURE: {

				this->data_array_u8[this->data_counter_u16] =
						this->recevied_data_dma_u8[i];

				this->package_id_u8 =
						this->data_array_u8[this->data_counter_u16];

				this->data_counter_u16++;
				this->capture_status_u8 = LENGTH_CAPTURE;

				break;
			}

			case LENGTH_CAPTURE: {

				this->data_array_u8[this->data_counter_u16] =
						this->recevied_data_dma_u8[i];

				this->data_counter_u16++;
				if (this->data_counter_u16 > 5) {

					this->payload_size_u8 =
							(this->data_array_u8[this->data_counter_u16 - 1]
									<< 8
									| this->data_array_u8[this->data_counter_u16
											- 2]);
					this->capture_status_u8 = PAYLOAD_CAPTURE;

				}

				break;
			}

			case PAYLOAD_CAPTURE: {

				this->data_array_u8[this->data_counter_u16] =
						this->recevied_data_dma_u8[i];

				this->data_counter_u16++;

				if (this->data_counter_u16 >= (this->payload_size_u8 + 6)) {

					this->data_counter_u16 = 0;
					this->capture_status_u8 = CHECKSUM_A_CAPTURE;
				}

				break;
			}

			case CHECKSUM_A_CAPTURE: {

				this->CK_A_u8 = this->recevied_data_dma_u8[i];

				this->capture_status_u8 = CHECKSUM_B_CAPTURE;

				break;
			}

			case CHECKSUM_B_CAPTURE: {

				this->CK_B_u8 = this->recevied_data_dma_u8[i];
				this->recevied_data_dma_u8[i] = 0;

				this->calculate_checksum_value(this->data_array_u8,
						(payload_size_u8 + 6));

				if (this->CK_A_u8 == calculated_CK_A_u8
						&& this->CK_B_u8 == this->calculated_CK_B_u8) {

					this->capture_status_u8 = HEADER_CAPTURE_FIRST;
					this->data_counter_u16 = 0;

					this->execute_nav_class_packages();

					memcpy(this->payload_array_u8, (this->data_array_u8 + 6),
							this->payload_size_u8);
					memset(this->data_array_u8, 0, CAPTURE_PACKAGE_SIZE);

				}

				else {

					this->capture_status_u8 = HEADER_CAPTURE_FIRST;
					this->package_class_u8 = 0;
					this->package_id_u8 = 0;
					this->payload_size_u8 = 0;

					memset(this->data_array_u8, 0, CAPTURE_PACKAGE_SIZE);
					this->data_counter_u16 = 0;

				}

				break;
			}

			default:
				this->capture_status_u8 = HEADER_CAPTURE_FIRST;
				break;

			} // switch

		} // for

	} // function

	// Parse gps message with State Machine Algorithm
	///////////////////////////////////////////////////////////



	void execute_nav_class_packages() {

		switch (this->package_id_u8) {

		case (PVT_MSG_ID): {
			this->execute_pvt_package();
			break;
		}

			/*case (other_package_you_want_get):{
			 *
			 * 	execute_other_package();
			 * 	break;
			 * }
			 */

		default:
			memset(this->payload_array_u8, 0 , PAYLOAD_SIZE);
			break;

		}

	}

	void execute_pvt_package() {

		/* PVT Message Structure :
		 * [0xB5][0x62][0x01][0x07][92 bytes][Payload][CK_A][CK_B]
		 */

		// Year -> byte offset : 4, size: 2 bytes
		this->nav_pvt_pavkage_st.year_u16 = (this->payload_array_u8[4])
				| (this->payload_array_u8[5] << 8);

		// Month -> byte offset: 6, size : 1 byte
		this->nav_pvt_pavkage_st.month_u8 = this->payload_array_u8[6];

		// Day -> bytes offset: 7, size: 1 byte
		this->nav_pvt_pavkage_st.day_u8 = this->payload_array_u8[7];

		// Hour -> bytes offset: 8, size: 1 byte
		this->nav_pvt_pavkage_st.hour_u8 = this->payload_array_u8[8];

		// Minutes -> bytes offset: 9, size: 1 byte
		this->nav_pvt_pavkage_st.min_u8 = this->payload_array_u8[9];

		// Seconds -> bytes offset: 10, size: 1 byte
		this->nav_pvt_pavkage_st.second_u8 = this->payload_array_u8[10];

		// Fix type -> bytes offset: 20, size: 1 bytes
		this->nav_pvt_pavkage_st.fix_type_u8 = this->payload_array_u8[20];

		// Longitude -> bytes offset: 24, size: 4 bytes
		this->nav_pvt_pavkage_st.longitude = ((this->payload_array_u8[24])
				| (this->payload_array_u8[25] << 8)
				| (this->payload_array_u8[26] << 16)
				| (this->payload_array_u8[27] << 24)) * 1e-7;

		// Latitude -> bytes offset: 28, size 4 bytes
		this->nav_pvt_pavkage_st.latitude = ((this->payload_array_u8[28])
				| (this->payload_array_u8[29] << 8)
				| (this->payload_array_u8[30] << 16)
				| (this->payload_array_u8[31] << 24)) * 1e-7;

		// Height above ellipsoid -> bytes offset: 32, size 4 bytes
		this->nav_pvt_pavkage_st.height_above_ellipsoid =
				((this->payload_array_u8[32])
						| (this->payload_array_u8[33] << 8)
						| (this->payload_array_u8[34] << 16)
						| (this->payload_array_u8[35] << 24)) * 1e-3;

		// Height above mean sea level -> bytes offset: 36, size 4 bytes
		this->nav_pvt_pavkage_st.height_above_mean_sea =
				((this->payload_array_u8[36])
						| (this->payload_array_u8[37] << 8)
						| (this->payload_array_u8[38] << 16)
						| (this->payload_array_u8[39] << 24)) * 1e-3;

	}

	void calculate_checksum_value(uint8_t *data_array, uint8_t size) {

		this->calculated_CK_A_u8 = 0;
		this->calculated_CK_B_u8 = 0;

		for (int i = 2; i < size; i++) {

			this->calculated_CK_A_u8 += data_array[i];
			this->calculated_CK_B_u8 += this->calculated_CK_A_u8;

			this->calculated_CK_A_u8 = this->calculated_CK_A_u8 & 0xFF;
			this->calculated_CK_B_u8 = this->calculated_CK_B_u8 & 0xFF;

		}
	}

	void start_communication_with_gps_via_dma() {

		HAL_UART_Receive_DMA(this->uartType, this->recevied_data_dma_u8,
		CAPTURE_PACKAGE_SIZE);
	}

};

typedef ublox_neo_m8n_gps_parameters *ublox_neo_m8n_gps_parameters_ptr;

#endif
