#pragma once

#include "pin.h"
#include "interrupt.h"

// по инициализации проема 8 байт 0xFF на первом проеме на двух независимых дверях. Водительская со жгутом выходов на входа.

struct Control {

};

struct In_id{
	Control control;
};

struct Error {
	bool overheat_fc : 1;
	bool overheat_c  : 1;
	bool over_HV     : 1;
	bool phase_break : 1;
	bool current_A   : 1;
	bool current_B   : 1;
	bool current_C   : 1;
	bool current_S   : 1;
};

struct State {
	bool on                 : 1;
	bool contactor          : 1;
	bool HV_low             : 1;
	bool voltage_board_low  : 1;
	bool voltage_board_high : 1;
	bool asin_drive         : 1;
	bool sin_drive          : 1;
	bool conditioner        : 1;
};

struct Error_2{
	bool limit_time : 1;
	bool total      : 1;
	uint8_t         : 6;
};

struct _ID_1 {
	uint16_t voltage_board;
	uint16_t voltage;
	uint16_t ARR;
	uint8_t voltage_phase;
	Error_2 error_2;
};

struct _ID_2 {
	State state;
	Error error;
	uint8_t temperature;
	uint8_t current;
	uint8_t current_drive;
	uint8_t PWM;
	uint16_t Kp;
};

struct Out_id{
	_ID_1 id_1;
	_ID_2 id_2;
};

template <class InID_t, class OutID_t>
class CAN : TickSubscriber
{

  Pin& rts;
//  Interrupt& tx_interrupt;
  Interrupt& rx_interrupt;

  CAN_TxHeaderTypeDef TxHeader;
  CAN_RxHeaderTypeDef RxHeader;

  uint8_t TxData[8];
  uint8_t RxData[8];

  uint32_t TxMailBox;

  uint8_t Data[31];
  uint8_t DataRx[31];

  uint32_t ID{1};
  const uint32_t ID_1{0x001};
  const uint32_t ID_2{0x002};

  uint16_t time{0};
  uint16_t time_refresh{0};

  bool work{false};

  uint8_t ToChar(uint8_t c) {
  	if  (0 <= c && c <= 9) {
  		return c + '0';
  	}else if ('a' <= c && c <= 'f') {
  		return c + 'a' - 10;
  	}else if (10 <= c && c <= 15) {
  		return c + 'A' - 10;
  	}
  	//return c + '0';
  }

  uint8_t FromChar(uint8_t c) {

  	if ('0' <= c && c <= '9') {
  		return c - '0';
  	}else if('a' <= c && c <= 'f'){
  		return c - 'a' + 10;
  	} else if('A' <= c && c <= 'F') {
  		return c - 'A' + 10;
  	}

  }

public:

  CAN(Pin& rts, Interrupt& rx_interrupt, uint16_t time_refresh)
  	  : rts{rts}
  	  , rx_interrupt{rx_interrupt}
  	  , time_refresh{time_refresh}
  {
//	  arInID[0] = arInID[1] = arInID[2] = arInID[3] = arInID[4] = arInID[5] = arInID[6] = arInID[7]= 0;
	  arOutID[0] = arOutID[1] = arOutID[2] = arOutID[3] = arOutID[4] = arOutID[5] = arOutID[6] = arOutID[7] = 0;
	  arOutID[8] = arOutID[9] = arOutID[10] = arOutID[11] = arOutID[12] = arOutID[13] = arOutID[14] = arOutID[15] = 0;
	  subscribed = false;
		TxHeader.DLC = 8;
		TxHeader.ExtId = 0;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.TransmitGlobalTime = DISABLE;
		if (time_refresh > 0)
			subscribe();
  }

  static const uint8_t InIDQty  = sizeof(InID_t);
  static const uint8_t OutIDQty = sizeof(OutID_t);

  union {
	InID_t inID;
    uint8_t arInID[InIDQty];
  };

  union {
    OutID_t outID;
    uint8_t arOutID[OutIDQty];
  };

  using Parent = CAN;

  struct can_rx_interrupt : Interrupting
  {
	  Parent& parent;
      can_rx_interrupt (Parent& parent) : parent(parent) {
          parent.rx_interrupt.subscribe (this);
      }
      void interrupt() {parent.receive();}
  }can_rx_{ *this };


  void transmit_ID_1(){
	  TxHeader.StdId = ID_1;
		TxData[0] = arOutID[1];
		TxData[1] = arOutID[0];
		TxData[2] = arOutID[3];
		TxData[3] = arOutID[2];
		TxData[4] = arOutID[5];
		TxData[5] = arOutID[4];
		TxData[6] = arOutID[6];
		TxData[7] = arOutID[7];
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox);
  }

  void transmit_ID_2(){
  		TxHeader.StdId = ID_2;
  		TxData[0] = arOutID[8];
  		TxData[1] = arOutID[9];
  		TxData[2] = arOutID[10];
  		TxData[3] = arOutID[11];
  		TxData[4] = arOutID[12];
  		TxData[5] = arOutID[13];
  		TxData[6] = arOutID[15];
  		TxData[7] = arOutID[14];
  		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox);
  }

  void receive(){
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);

//		switch(RxHeader.StdId) {
//			case 0xDC:
//				arInID[0] = RxData[0];
//				arInID[1] = RxData[1];
//				inID.initial = RxData[7];
//				start_transmit();
//				break;
//			case 0xAA:
//				outID.state.open_driver = RxData[0] & (1 << 4);
//				break;
//		}
	}

  bool is_work(){ return work; }

  void start_transmit() {
		if (not work) {
			work = true;
			if (time_refresh > 0)
				subscribe();
		}
  }

  void stop_transmit() { unsubscribe(); work = false; }

  void notify() {
	  if (time++ >= (time_refresh)) {
		  rts = true;
		  time = 0;
		  switch(ID) {
		  case 0x001:
			  transmit_ID_1();
			  ID = ID_2;
			  break;
		  case 0x002:
			  transmit_ID_2();
			  ID = ID_1;
		 	  break;
		  }
		  rts = false;
	  }
//	  if(inID.control.on_off) stop_transmit();
  }

};


//Interrupt interrupt_can_tx;
Interrupt interrupt_can_rx;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  interrupt_can_rx.interrupt();
}

