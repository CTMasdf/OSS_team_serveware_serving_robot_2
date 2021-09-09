/*
 * serve_2_robot.c
 *
 * Created: 2021-07-31 오후 4:59:32
 * Author : CTM
 */ 


#include <avr/io.h>				//기본 입출력 헤더파일 | Basic input/output header file
#include <util/delay.h>			//딜레이 헤더파일 | delay header file
#include <avr/interrupt.h>		//인터럽트 헤더파일 | interrupt header file
#include <string.h>				//문자열 처리 헤더파일 | String processing header file
#include <avr/eeprom.h>			//EEPROM 헤더파일 | EEPROM header file

/*************************출력핀 GPIO define | Output pin GPIO define*******************************/
#define		DATA_ON					PORTD = PORTD | 0x08;	//ic4094 DATA핀을 1로 출력 | output ic4094 DATA pin as 1
#define		CLK_ON					PORTD = PORTD | 0x10;	//ic4094 CLK핀을 1로 출력 | output ic4094 CLK pin to 1
#define		FND1_OFF				PORTD = PORTD | 0x20;	//FND1번 핀 1출력
#define		FND2_OFF				PORTD = PORTD | 0x40;	//FND2번 핀 1출력
#define		PUSH_BUTTON_LED_ON		PORTA = PORTA | 0x01;	//LED 버튼의 LED를 1로 출력 | Output the LED of the LED button as 1.

#define		DATA_OFF				PORTD = PORTD &~ (0x08);	//ic4094 DATA핀을 0으로 출력 | output ic4094 DATA pin as 0
#define		CLK_OFF					PORTD = PORTD &~ (0x10);	//ic4094 CLK핀을 0으로 출력 | output ic4094 CLK pin as 0
#define		FND1_ON					PORTD = PORTD &~ (0x20);	//FND1번 핀 1출력
#define		FND2_ON					PORTD = PORTD &~ (0x40);	//FND2번 핀 1출력
#define		PUSH_BUTTON_LED_OFF		PORTA = PORTA &~ (0x01);	//LED 버튼의 LED를 0으로 출력 | Output the LED of the LED button as 0



volatile unsigned int fndc, fndc2, fnd_com;						//4015에 static 방식으로 연결된 FND, LED 카운터 변수 | FND statically connected to 4094, LED counter variable
volatile unsigned int num1, num10;								//1의 자리, 10의 자리 변수 | 1's place, 10's place variable
volatile int blue_led_dat, red_led_dat, blue_led, red_led;		//파란색 LED, 빨간색 LED 변수 | Blue LED, Red LED Variable

//푸쉬 버튼, 푸쉬 버튼_flag, 버튼 led 변수 | pushbutton, pushbutton_flag, button led variable.
volatile unsigned int button, button_f, button_start, button_led, button_ledc, button_led_f, tick;

/*엔코더 변수 | encoder variable*/
volatile unsigned int seg_en1, seg_en2, left_en1, left_en2, right_en1, right_en2,
s_en1, s_en2, l_en1, l_en2, r_en1, r_en2,
s_en1_c, s_en2_c, l_en1_c, l_en2_c, r_en1_c, r_en2_c,
left_number = 0, right_number = 0;

volatile int number = 1;	//FND 숫자(테이블) 초기값은 1이다. | The initial value of FND number (table) is 1.
volatile int tick_stop, stop_flag;	//DC모터 함수의 모터정지 변수 | Motor stop variable of DC motor function

//FND 배열 {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, -, , E, r}
//FND array {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, -, , E, r}
volatile int num[14] = {0x003f, 0x0006, 0x005b, 0x004f, 0x0066, 0x006d, 0x007c, 0x0007, 0x007f, 0x0067, 0x0040, 0, 0x0079, 0x0050};

volatile int run_data[49];		//[거리측정 배열] | [distance measurement arrangement]
volatile int move_data[49];		//[방향전환 배열] | [redirection array]
volatile int table_data[100];	//[테이블 데이터] | [table data]

volatile unsigned int set, set_table_finish = 0,
move_cnt, run_data_cnt, start_data_cnt, set_table_arrive;	//설정동작 변수 | Setting action variable

volatile unsigned int serving_start, run_flag, run_switch;	//서빙 동작모드 변수 | Serving Mode Variables
volatile int flag, point, move_flag, cnt1, next_move, serving_complete, delay_cnt, delay_flag;	//서빙 동작모드 변수 | 서빙 동작모드 변수

volatile unsigned int error, error_cnt, dc_stop,  serving_move_cnt;	//에러동작_변수 | error action_variable
volatile int button_cnt;	//테이블값 초기화 변수 | table value initialization variable

/*EEPROME 쓰기 함수 선언 | EEPROM write function declaration*/
void EEPROM_write(uint16_t uiAddress, uint8_t ucData){
	/* Wait for completion of previous write */
	while(EECR & (1<<EEWE));
	/* Set up address and data registers */
	EEAR = uiAddress;EEDR = ucData;
	/* Write logical one to EEMWE */
	EECR |= (1<<EEMWE);
	/* Start eeprom write by setting EEWE */
	EECR |= (1<<EEWE);
}

/*EEPROME 읽기 함수 선언 | EEPROM read function declaration*/
uint8_t EEPROM_read(uint16_t uiAddress){
	/* Wait for completion of previous write */
	while(EECR & (1<<EEWE));
	/* Set up address register */
	EEAR = uiAddress;
	/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	/* Return data from data register */
	return EEDR;
}

/*버튼 함수 선언 | button function declaration*/
void button_control(){
	if(button == 0){	//버튼을 누르고 있을 때 | when the button is pressed
		button_cnt++;	//버튼 카운트 변수 증가 | increment button count variable
		switch(button_cnt){
			case 3000:	//3초후 | after 3 seconds
			EEPROM_write(number * 100, 0);	//현재 테이블값 초기화 | Initialize the current table value
			break;
		}
	}
	
	if(button && !button_f){	//button이 1이고 button_f가 1이 아닐 때 (버튼을 눌렀다 땠을 때) | When button is 1 and button_f is not 1 (when button is pressed and released)
		button_f = 1;	//button_f = 1;
		tick++;			//tick 1ms씩 증가 | tick increments by 1ms
		if(tick<50){	//시간 딜레이 (버튼을 눌렀다 뗐을 때 1번 이상 증가하지 않도록 하는 부분) | Time delay (the part that does not increase more than 1 time when the button is pressed and released)
			button_cnt = 0;	//버튼 카운트 변수 0으로 초기화 | Initialize button count variable to 0
			if(button_cnt < 3000){	//버튼을 3초이상 누르지 않았을 때 | When the button is not pressed for more than 3 seconds
				
				/*버튼을 눌렀다 뗐을 때 동작 여기에 코드 적기 | Action when button is pressed and released Write code here*/
				if(set == 0){	//서빙 동작모드일 때 | When in serving mode
					volatile int8_t table_eep_read = EEPROM_read(number * 100);	//현재 테이블값 읽기 | read current table value
					table_data[number] = number;	//테이블 데이터 배열  = number; | table data array = number;
					
					if(table_data[number] == table_eep_read){	//테이블 데이터 배열값이 eeprom에서 읽어온 값과 같을 때 | If table data array value is same as value read from eeprom
						/*서빙모드_시작 | Serving Mode_Start*/
						left_number = 0;		//왼쪽 모터 엔코더 값을 0으로 초기화 시킨다. | Initialize the left motor encoder value to 0.
						right_number = 0;		//오른쪽 모터 엔코더 값을 0으로 초기화 시킨다. | Initialize the right motor encoder value to 0.
						serving_start = 1;		//서빙을 시작한다. | Start serving.
						set_table_finish = 1;	//테이블 설정을 완료시켰다. | Table setup is complete.
					}
					else if(table_data[number] != table_eep_read){	//테이블 데이터 배열값이 eeprom에서 읽어온 값과 다를 때 | If the table data array value is different from the value read from eeprom
						/*에러 (선택한 테이블의 경로가 설정되어 있지 않기 때문에 동작) | Error (because path to selected table is not set)*/
						error = 1;	//에러 시작 | error start
					}
					if(serving_complete == 1){	//테이블에 도착했을 때 버튼을 눌렀다 떼면 | When you get to the table, press and release the button.
						serving_complete = 2;	//복귀 동작 시작 | start of return movement
						right_number = 1;		//오른쪽 모터 엔코더값 1로 초기화 | Reset right motor encoder value 1
						left_number = 1;		//왼쪽 모터 엔코더값 1로 초기화 | Reset left motor encoder value 1
					}
				}
				else if(set == 1){	//설정 동작이 시작했을 때 | When the setup operation starts
					set_table_finish = 1;	//테이블 설정을 완료하고 경로설정으로 이동한다. | Finish setting up the table and go to Set Path.
					if(set_table_arrive == 1){	//설정 동작이 완료됐을 때 | When setup is complete
						/*설정동작 완료 코드 여기에 적기 | Write your setup job completion code here.*/
						int i;
						for(i = 0; i < 3; i++){	//현재 방향이동 배열에서 설정동작 완료 표시('6'으로 표시)를 해주고 방향이동 변수를 0으로 초기화 시켜주기 위해 사용 | Used to indicate that the operation set in the current direction array (indicated by '6') has been completed and to initialize the direction variable to zero.
							switch(i){
								case 1:
								move_data[move_cnt] = 6;	//방향이동 배열에서 설정동작을 완료했다는 표시 | Indicates that the setup operation has been completed in the directional movement array.
								table_data[number] = number;	//몇 번 테이블을 설정했는지 저장 | Save how many times a table has been set
								/*EEPROM 쓰기 | EEPROM_WRITE*/
								EEPROM_write(number * 100, number);	//현재 테이블값 eeprom에 쓰기 | Write to current table value eeprom
								int j, k;
								for(j = 0; j <= run_data_cnt; j++){	//거리측정한 갯수만큼 증가 | increase by the number of measurement distances
									EEPROM_write((j + 51) + (number * 100), run_data[j]);	//거리측정 데이터 저장 | Store distance measurement data
								}
								for(k = 0; k <= move_cnt; k++){		//방향전환한 갯수만큼 증가 | Increases by the number of redirects
									EEPROM_write((k + 1) + (number * 100), move_data[k]);	//방향이동 데이터 저장 | Storing direction movement data
								}
								break;
								case 2:
								move_cnt = 0;			//방향이동 변수를 0으로 초기화 시킨다. | Initialize the direction movement variable to 0.
								run_data_cnt = 0;		//거리측정 변수를 0으로 초기화 시킨다. | Initialize the distance measurement variable to 0.
								serving_start = 0;		//서빙모드를 0으로 초기화 시킨다. | Initialize the serving mode to 0.
								number = 0;				//fnd 숫자를 0으로 초기화 시킨다. | Initialize the fnd number to 0.
								fnd_com = 0;			//fnd를 점멸하지 않는다. | Don't flash fnd.
								red_led = 0;			//빨간색 LED_OFF | Red LED_OFF
								blue_led = 0;			//파란색 LED_OFF	| Blue LED_OFF
								set = 0;				//설정동작에서 나온다. | It comes out of the setup action.
								set_table_finish = 0;	//테이블 설정으로 돌아간다. | Return to table settings.
								set_table_arrive = 0;	//카운터에서 테이블까지 도착한 값을 0으로 초기화 시킨다. | Values ​​arriving from the counter to the table are initialized to zero.
								break;
							}
						}
					}
				}
			}
		}
	}
	else if(!button && button_f){	//button이 1이 아니고 button_f가 1일 때 | When button is not 1 and button_f is 1
		button_f = 0;	//button_f = 0;
		tick = 0;		//tick = 0;
	}
}
/*엔코더 함수 선언 | Encoder function declaration*/
void encoder(){
	/************테이블_설정_엔코더 | table_set_encoder***********/
	if(seg_en1 == 0 && seg_en2 != 0){	//seg_en1이 0, seg_en2가 0이 아닐 때 | If seg_en1 is 0 and seg_en2 is non-zero
		s_en1 = 1;						//en1 = 1;
		s_en2 = 1;						//en2 = 1;
	}
	else if(seg_en1 != 0 && seg_en2 == 0){	//seg_en1이 0이 아니고 seg_en2가 0일 때 | If seg_en1 is not 0 and seg_en2 is 0
		s_en1 = 0;						//en1 = 0;
		s_en2 = 1;						//en1 = 1;
	}
	
	/************왼쪽_모터_엔코더 | left_motor_encoder***********/
	if(left_en1 == 0 && left_en2 != 0){	//left_en1이 0, left_en2가 0이 아닐 때 | If left_en1 is 0 and left_en2 is non-zero
		l_en1 = 1;						//l_en1 = 1;
		l_en2 = 1;						//l_en2 = 1;
	}
	else if(left_en1 != 0 && left_en2 == 0){	//left_en1이 0이 아니고 left_en2가 0일 때 | When left_en1 is not 0 and left_en2 is 0
		l_en1 = 0;						//l_en1 = 0;
		l_en2 = 1;						//l_en1 = 1;
	}
	
	/************오른쪽_모터_엔코더 | right_motor_encoder***********/
	if(right_en1 == 0 && right_en2 != 0){	//right_en1이 0, right_en2가 0이 아닐 때 | If right_en1 is 0 and right_en2 is non-zero
		r_en1 = 1;						//r_en1 = 1;
		r_en2 = 1;						//r_en2 = 1;
	}
	else if(right_en1 != 0 && right_en2 == 0){	//right_en1이 0이 아니고 right_en2가 0일 때 | If right_en1 is not 0 and right_en2 is 0
		r_en1 = 0;						//r_en1 = 0;
		r_en2 = 1;						//r_en1 = 1;
	}
	/************테이블_설정_엔코더_제어 | table_set_encoder_control***********/
	if (seg_en1 != 0 && seg_en2 != 0 && s_en2 == 1){
		s_en2 = 0;
		if(s_en1 == 0){
			/*이 안에 엔코더 제어 코드 적기 (역방향) | Write your encoder control code here (reverse).*/
			if(serving_start == 0 || set_table_finish == 0){	//서빙모드가 시작되지 않을 때 또는 테이블 설정이 완료되지 않았을 때 | Serving mode does not start or table setup is not complete.
				number--;	//테이블 번호 감소. | Decrease table number.
			}
		}
		else {
			/*이 안에 엔코더 제어 코드 적기 (정방향) | Write the encoder control code here (forward).*/
			if(serving_start == 0 || set_table_finish == 0){	//서빙모드가 시작되지 않을 때 또는 테이블 설정이 완료되지 않았을 때 | Serving mode does not start or table setup is not complete.
				number++;	//테이블 번호 증가. | increase table number.
			}
		}
	}
	/*엔코더를 돌렸을 때 50초과, 1미만을 넘지 안도록 하는 소스(테이블 갯수 50개) | Sources less than 1 but not more than 50 when the encoder is turned (50 tables)*/
	if(number >= 50)			//number가 50보다 크거나 같을 때 | When the number is greater than or equal to 50
	number = 50;			//number = 50;
	else if(number <= 1)		//number가 0보다 작거나 같을 때 | When a number is less than or equal to 0
	number = 1;				//number = 0;
		
	/************왼쪽_모터_엔코더_제어 | left_motor_encoder_control***********/
	if (left_en1 != 0 && left_en2 != 0 && l_en2 == 1){
		l_en2 = 0;
		if(l_en1 == 0){
			/*이 안에 엔코더 제어 코드 적기 (역방향) | Write your encoder control code here (reverse).*/
			if(set == 1 && set_table_finish == 1){	//설정동작 시작 && 테이블을 결정 완료 | Start setting up && table determination complete
				if(UDR == '=' || UDR == '<'){		//블루투스 모듈에서 신호가 '전진' 또는 '좌회전' 일 때 | When the signal from the Bluetooth module is 'forward' or 'turn left'
					left_number++;					//left_number 증가 (왼쪽 모터 엔코더를 돌렸을 때) | increment left_number (when left motor encoder rotates)
					run_data[run_data_cnt] = left_number;	//거리 측정 배열에 왼쪽 모터 엔코더값 저장 | Storing Left Motor Encoder Values ​​in Distance Measure Array
				}
			}
			if(serving_start == 1){	//서빙모드일 때 | When in serving mode
				if(serving_move_cnt == 2 || serving_move_cnt == 3){	//전진 또는 좌회전일 때 | When making a forward or left turn
					left_number++;	//left_number 증가 (왼쪽 모터 엔코더를 돌렸을 때) | increment left_number (when left motor encoder rotates)
				}
			}
		}
		else {
			/*이 안에 엔코더 제어 코드 적기 (정방향) | Write the encoder control code here (forward).*/
			if(set == 1 && set_table_finish == 1){	//설정동작 시작 && 테이블을 결정 완료 | Start setting up && table determination complete
				if(UDR == '=' || UDR == '<'){		//블루투스 모듈에서 신호가 '전진' 또는 '좌회전' 일 때 | When the signal from the Bluetooth module is 'forward' or 'turn left'
					left_number++;					//left_number 증가 (왼쪽 모터 엔코더를 돌렸을 때) | increment left_number (when left motor encoder rotates)
					run_data[run_data_cnt] = left_number;	//거리 측정 배열에 왼쪽 모터 엔코더값 저장 | Storing Left Motor Encoder Values ​​in Distance Measure Array
				}
			}
			if(serving_start == 1){	//서빙모드일 때 | When in serving mode
				if(serving_move_cnt == 2 || serving_move_cnt == 3){	//블루투스 모듈에서 신호가 '전진' 또는 '좌회전' 일 때 | When the signal from the Bluetooth module is 'forward' or 'turn left'
					left_number++;					//left_number 증가 (왼쪽 모터 엔코더를 돌렸을 때)
				}
			}
		}
	}
	
	/************오른쪽_모터_엔코더_제어 | right_motor_encoder_control***********/
	if (right_en1 != 0 && right_en2 != 0 && r_en2 == 1){
		r_en2 = 0;
		blue_led = 1;
		if(r_en1 == 0){
			/*이 안에 엔코더 제어 코드 적기 (역방향) | Write your encoder control code here (reverse).*/
			if(set == 1 && set_table_finish == 1){	//설정동작 시작 && 테이블을 결정 완료 | Start setting up && table determination complete
				if(UDR == '+' || UDR == '>'){		//블루투스 모듈에서 신호가 '후진' 또는 '우회전' 일 때 | When the signal from the Bluetooth module is 'reverse' or 'turn right'
					right_number++;					//right_number 증가 (오른쪽 모터 엔코더를 돌렸을 때) | increment right_number (when the right motor encoder rotates)
					run_data[run_data_cnt] = right_number;	//거리 측정 배열에 오른쪽 모터 엔코더값 저장 | Storing Right Motor Encoder Values ​​in Distance Measure Array
				}
			}
			if(serving_start == 1){	//서빙모드일 때 | When in serving mode
				if(serving_move_cnt == 1 || serving_move_cnt == 4){	//후진 또는 우회전일 때 | When reversing or turning right
					right_number++;					//right_number 증가 (오른쪽 모터 엔코더를 돌렸을 때) | increment right_number (when the right motor encoder rotates)
				}
			}
		}
		else {
			/*이 안에 엔코더 제어 코드 적기 (정방향) | Write the encoder control code here (forward).*/
			if(set == 1 && set_table_finish == 1){	//설정동작 시작 && 테이블을 결정 완료 | Start setting up && table determination complete
				if(UDR == '+' || UDR == '>'){		//블루투스 모듈에서 신호가 '후진' 또는 '우회전' 일 때 | When the signal from the Bluetooth module is 'reverse' or 'turn right'
					right_number++;					//right_number 증가 (오른쪽 모터 엔코더를 돌렸을 때) | increment right_number (when the right motor encoder rotates)
					run_data[run_data_cnt] = right_number;	//거리 측정 배열에 오른쪽 모터 엔코더값 저장 | Storing Right Motor Encoder Values ​​in Distance Measure Array
				}
			}
			if(serving_start == 1){	//서빙모드일 때 | When in serving mode
				if(serving_move_cnt == 1 || serving_move_cnt == 4){	//후진 또는 우회전일 때 | When reversing or turning right
					right_number++;
				}									//right_number 증가 (오른쪽 모터 엔코더를 돌렸을 때) | increment right_number (when the right motor encoder rotates)
			}
		}
	}
}

/*ic4015 함수 선언 | ic4015 function declaration*/
void ic4015(int data){
	int i;
	for(i = 0; i < 16; i++){
		if(data&(0x8000 >> i)){
			DATA_ON;	
		}
		else
			DATA_OFF;
		CLK_ON;
		_delay_us(1);
		CLK_OFF;
	}
}
/*FND_LED 함수 선언*/
void FND_LED(){
	
	/****************카운터 증가 | counter increment*******************/
	button_ledc++;	//버튼 LED변수 카운터 증가 | Button LED Variable Counter Increment
	fndc++;			//fndc변수가 1ms마다 증가 | The fndc variable is incremented every 1ms.
	fndc2++;		//fndc2변수가 1ms마다 증가 | The fndc2 variable is incremented every 1ms.
	
	/*****************FND에 에러표시 | Show error on FND*********************/
	if(error == 0){	//error가 동작하지 않을 때(fnd에 테이블 번호를 출력) | When there is no error (output table number as fnd)
		num10 = number / 10;	//num10 = number / 10 이다.(변수 number의 10의 자리) | num10 = number / 10. (10 digits of variable number)
		num1 = number % 10;		//num1 = number % 10 이다. (변수 number의 1의 자리) | number 1 = number %10. (magnification of 1)
	}
	else{	//error가 동작될 때(fnd에 'Er'를 띄운다.) | When an error occurs ('Er' is displayed in fnd)
		error_cnt++;	//error_cnt 변수 1ms마다 증가. | The error_cnt variable is incremented every 1ms.
		if(error_cnt >= 500){	//500ms이상일 때 | When more than 500ms
			error = 0;		//에러종료 | end of error
			error_cnt = 0;	//에러 카운트 종료 | end of error count
		}
		num10 = 12;	//FND 10의 자리 ('E'출력) | FND 10's digit ('E' output)
		num1 = 13;	//FND 1의 자리 ('r'출력) | FND 1's digit ('r' output)
	}
	
	/*******************FND 제어 | FND control*************************************/
	if(fndc2 >= 1000){	//fndc가 1000ms이상일 때(1초 주기) | When fndc is over 1000ms(1 second cycle)
		fndc2 = 0;		//fndc2 = 0;
	}
	
	if(fndc2 < 500){	//fndc2가 500ms보다 작을 때(0.5초씩 점멸하기 위해 사용) | When fndc2 is less than 500 ms (used to blink in 0.5 second increments)
		switch(fndc){	//fndc일 때
			//ic4094(10의 자리 데이터 + 10의 자리 FND_ON + 빨간색 LED 데이터 + 파란색 LED 데이터) | ic4094 (10's digit data + 10's FND_ON + red LED data + blue LED data)
			case 1: FND1_OFF; FND2_OFF; break;
			case 3: ic4015(num[num10] + red_led_dat + blue_led_dat); break;
			//ic4094(1의 자리 데이터 + 1의 자리 FND_ON + 빨간색 LED 데이터 + 파란색 LED 데이터) | ic4094 (1 digit data + 1 digit FND_ON + red LED data + blue LED data)
			case 5: FND1_ON; FND2_OFF; break;
			case 7: FND1_OFF; FND2_OFF; break;
			case 9: ic4015(num[num1] + red_led_dat + blue_led_dat); break;
			case 11: FND1_OFF; FND2_ON; break;
			case 15: fndc = 0; break;	//fndc = 0;
		}
	}
	else if(fndc2 >= 500){	//fndc2가 500ms 이상일 때 | If fndc2 is greater than 500ms
		if(fnd_com == 0){	//fnd를 0.5초씩 점멸하지 않을 때 | When fnd does not blink for 0.5 seconds
			switch(fndc){	//fndc
				//ic4094(10의 자리 데이터 + 10의 자리 FND_ON + 빨간색 LED 데이터 + 파란색 LED 데이터) | ic4094 (10's digit data + 10's FND_ON + red LED data + blue LED data)
				case 1: FND1_OFF; FND2_OFF; break;
				case 3: ic4015(num[num10] + red_led_dat + blue_led_dat); break;
				//ic4094(1의 자리 데이터 + 1의 자리 FND_ON + 빨간색 LED 데이터 + 파란색 LED 데이터) | ic4094 (1 digit data + 1 digit FND_ON + red LED data + blue LED data)
				case 5: FND1_ON; FND2_OFF; break;
				case 7: FND1_OFF; FND2_OFF; break;
				case 9: ic4015(num[num1] + red_led_dat + blue_led_dat); break;
				case 11: FND1_OFF; FND2_ON; break;
				case 15: fndc = 0; break;	//fndc = 0;
			}
		}
		else if(fnd_com == 1){
			switch(fndc){	//fndc일 때
				case 1: FND1_OFF; FND2_OFF; break;
				case 3: ic4015(num[11] + red_led_dat + blue_led_dat); break;
				//ic4094(1의 자리 데이터 + 1의 자리 FND_ON + 빨간색 LED 데이터 + 파란색 LED 데이터) | ic4094 (1 digit data + 1 digit FND_ON + red LED data + blue LED data)
				case 5: FND1_ON; FND2_OFF; break;
				case 7: FND1_OFF; FND2_OFF; break;
				case 9: ic4015(num[11] + red_led_dat + blue_led_dat); break;
				case 11: FND1_ON; FND2_OFF; break;
				case 15: fndc = 0; break;	//fndc = 0;
			}
		}
	}
	
	/*****************버튼 led 500ms 주기 blank 제어*********************/
	if(button_ledc >= 1000){	//button_ledc가 1000ms 이상이면 (0.5초 마다 led가 깜빡여야 하기때문에 1초 주기를 만듬) | When button_ledc is greater than 1000ms (the LED should blink every 0.5 seconds, so let's make it a 1 second cycle)
		button_ledc = 0;	//button_ledc = 0;
	}
	if(button_ledc == 0){	//버튼 led 카운터가 500 미만 일 때 | When the button led counter is less than 500
		switch(button_led){
			case 0: PUSH_BUTTON_LED_OFF break;	//button_led가 0일 때 버튼 led off | button is off when button_led is 0
			case 1: PUSH_BUTTON_LED_ON break;	//button_led가 1일 때 버튼 led on | button is on when button_led is 1
		}
	}
	else if(button_ledc == 500){	//버튼 led 카운터가 500ms 일 때 | When the button led counter is 500ms
		if(button_led == 1 && button_led_f == 0){	//button_led가 1이고 button_led_f가 0이면 | When button_led is 1 and button_led_f is 0
			PUSH_BUTTON_LED_ON		//Button led on
		}
		else if(button_led_f == 1){	//button_led_f가 1이면
			PUSH_BUTTON_LED_OFF		//Button led off
		}
	}
	
	/****************************파란색, 빨간색 led 제어 | Blue, red led control************************************/
	if(red_led == 1)			//만약 빨간색 LED가 1일 때 | If the red LED is 1
	red_led_dat = 0x0100;	//빨간색 LED 데이터는 0x0800이다. | Red LED data is 0x0100.
	else						//만약 빨간색 LED가 1이 아닐 때 | If the red LED is not 1
	red_led_dat = 0;		//빨간색 LED 데이터는 0이다. | Red LED data is zero.
	if(blue_led == 1)			//만약 파란색 LED가 1일 때 | If the blue LED is 1
	blue_led_dat = 0x0200;	//파란색 LED 데이터는 0x0400이다. | The blue LED data is 0x0200.
	else						//만약 파란색 LED가 1이 아닐 때 | If the blue LED is not 1
	blue_led_dat = 0;		//파란색 LED 데이터는 0이다. | Blue LED data is 0.
}

/*DC모터 함수 선언 | DC motor function declaration*/
void DC_MOTOR(){
	if((set == 1 && set_table_finish == 1) || serving_start == 1){	//설정모드에 진입하고 테이블 설정을 마칠때 | When entering setting mode and setting the finishing table
		
		if(serving_complete == 0 || serving_complete == 2){	//테이블에 이동중일 때 또는 테이블에서 되돌아올 때 | when moving to or returning from a table
			if(UDR == '5' && !stop_flag){		    //모터정지 신호가 입력되면 | When a motor stop signal is input
				//stop_flag는 변수가 1씩 증가하게 해주는 flag 변수 | stop_flag is a flag variable that increments the variable by 1.
				stop_flag = 1;		//stop_flag = 1;
				tick_stop++;		//tick_stop변수 1ms씩 증가 | The tick_stop variable is incremented by 1 ms.
				
				if(tick_stop < 50){		//한 번만 증가하기 위해 50ms 딜레이를 줌 | Provides 50ms delay for one increment
					run_data_cnt++;		//다음동작 준비(거리) | Prepare for the next move (distance)
					move_cnt++;			//다음동작 준비(방향전환) | Prepare for the next operation (change direction).
				}
			}
			else if(UDR != '5' && stop_flag){	//모터정지 신호가 아니면 | If there is no motor stop signal
				stop_flag = 0;	//stop_flag = 0;
				tick_stop = 0;	//tick_stio = 0;
			}
			
			if(UDR == '5' || serving_move_cnt == 0){	//설정동작에서 모터정지 신호수신 또는 서빙모드에서 모터정지 신호를 읽어오면 | When receiving motor stop signal in set operation or reading motor stop signal in serving mode,
				PORTB = 0;	//(GPIO_control) 모터 정지 | motor stop
			}
			
			if(UDR == '+' || serving_move_cnt == 1){		//로봇 후진 | back up a robot
				if(serving_start != 1){			//서빙동작이 동작하고 있지 않을 때 | When no scan jobs are running
					move_data[move_cnt] = 1;	//move_data[move_cnt]에 1값을 (후진)저장 | Store value 1 in move_data[move_cnt] (reverse)
				}
				PORTB = 0x06;	//(GPIO_control) 모터 후진 | back up a motor
			}
			if(UDR == '=' || serving_move_cnt == 2){		//로봇 전진 | robot forward
				if(serving_start != 1){			//서빙동작이 동작하고 있지 않을 때 | When no serving jobs are running
					move_data[move_cnt] = 2;	//move _data[move_cnt]에 2값을 (전진)저장 | Store 2 values ​​(forward) in move_data[move_cnt]
				}
				PORTB = 0x09;	//(GPIO_control) 모터 전진 | motor forward
			}
			if(UDR == '<' || serving_move_cnt == 3){		//로봇 좌회전 | robot turn left
				if(serving_start != 1){			//서빙동작이 동작하고 있지 않을 때 | When no serving jobs are running
					move_data[move_cnt] = 3;	//move _data[move_cnt]에 3값을 (좌회전)저장 | Save 3 values ​​in move_data[move_cnt] (turn left)
				}
				PORTB = 0x02;	//(GPIO_control) 모터 좌회전 | motor turn left
			}
			if(UDR == '>' || serving_move_cnt == 4){		//로봇 우회전 | robot turn right
				if(serving_start != 1){			//서빙동작이 동작하고 있지 않을 때 | When no serving jobs are running
					move_data[move_cnt] = 4;	//move _data[move_cnt]에 4값을 (우회전)저장 | Save 4 values ​​in move_data[move_cnt] (turn right)
				}
				PORTB = 0x04;	//(GPIO_control) 모터 우회전 | motor turn right   
			}
		}
	}
}
/*설정동작 및 서빙동작 함수 선언 | Declare setting action and serving action function*/
void mode_setting_move(){
	if(UDR == '1'){	//블루투스 신호가 1일 때 설정진입 | Enter setting when Bluetooth signal is 1
		set = 1;	//세팅변수 값을 1로 설정 | set variable value to 1
	}
	else if(UDR == '0'){	//블루투스 신호가 0일 때 설정취소 | Cancel setting when Bluetooth signal is 0
		set = 0;			//세팅변수 값을 0으로 설정 | Set the set variable value to 0
		blue_led = 0;		//파란색 LED ON | Blue LED ON
	}
	
	/*설정모드 진입 | Enter setting mode*/
	if(set == 1){
		serving_start = 2;				//설정동작 중 서빙동작에서 동작되는 엔코더 설정을 하지 않도록 한다. | Do not set the encoder that does the serving operation during the setting operation.
		button_led = 1;					//버튼 LED_ON | Button LED ON
		
		if(set_table_arrive == 1){		//설정동작에서 테이블에 도착했을 때 | When you arrive at the table of setting work
			blue_led = 0;				//파란색 LED_OFF | Blue LED OFF
		}
		else if(set_table_arrive != 1){	//설정동작에서 테이블에 도착한 신호를 받지 않았을 때 | When the signal arriving at the table is not received in the setting operation
			blue_led = 1;				//파란색 LED_ON | Blue LED OFF
		}
		
		if(set_table_finish == 0)		//설정할 테이블이 결정되지 않으면 | When the table to set is not determined
		button_led_f = 1;			//버튼 led를 500ms 마다 깜빡인다. | Button LED blinks every 500ms.
		
		else								//테이블이 결정되면 | When the table is decided
		button_led_f = 0;				//버튼 led를 500ms 마다 깜빡이지 않는다. | The button LED doesn't blink every 500ms.
		if(set_table_finish == 1){			//테이블 번호를 설정한 후 | After setting the table number
			if(UDR == '2'){					//카운터에서 출발해서 테이블에 도착했을 때 (블루투스 신호 2)| When you leave the counter and arrive at the table (Bluetooth signal 2)
				move_data[move_cnt] = 5;	//현재 이동방향 배열값을 5로 바꾼다. | Change the current movement direction array value to 5.
				run_data[run_data_cnt] = 0; //현재 거리 배열값을 0으로 바꾼다. | Change the current distance array value to 0.
				fnd_com = 1;				//FND를 500ms 마다 깜박인다. | FND blinks every 500ms.
				set_table_arrive = 1;		//설정동작에서 카운터에서 테이블까지 도착했다는 신호. | Signals the arrival of the counter from the counter to the table in the setup operation.
				red_led = 1;				//빨간색 LED_ON | Red LED ON
			}
		}
	}
	
	/*서빙모드 | Serving mode*/
	if(set == 0){
		if(serving_start == 0){	//서빙을 하고 있지 않을 때 | when not serving
			button_led = 1;		//버튼 LED를 켠다. | Turn on the button LED.
			button_led_f = 1;	//버튼 LED를 500ms마다 깜박인다. | Button LED blinks every 500ms.
		}
		else if(serving_start == 1){	//서빙을 시작했을 때 | when you start serving
			if(serving_complete == 0){	//테이블에 도착하지 않았을 때 | When you do not arrive at the table
				button_led = 0;			//버튼 led를 끈다. | Turn off the button LED
				button_led_f = 0;		//버튼 led가 500ms마다 점멸하지 않는다. | The button LED doesn't blink every 500ms.
				blue_led = 1;			//파란색 LED ON | Blue LED ON
			}
			else if(serving_complete == 1){	//테이블에 도착했을 때 | when you get to the table
				button_led = 1;		//버튼 LED를 켠다. | Turn on the button LED.
				button_led_f = 1;	//버튼 LED를 500ms마다 깜박인다. | Button LED blinks every 500ms.
				blue_led = 0;		//파란색 LED OFF | Blue LED OFF
			}
			else {	//로봇이 테이블에 도착하고 손님이 물건을 받아서 버튼을 눌렀을 때 | When the robot arrives at the table, the customer picks up an item and presses a button.
				button_led = 0;		//버튼 LED 끄기 | Buton LED off.
				button_led_f = 0;	//버튼 LED가 0.5초 마다 깜박이지 않는다. | Button LED does not blink every 0.5 seconds.
				red_led = 1;		//빨간색 LED를 켠다. | Red LED on.
				fnd_com = 1;		//FND를 500ms마다 점멸한다. | FND blinks every 500ms.
			}
			switch(move_flag){	//서빙동작 중 EEPROM 함수 읽기 변수 | EEPROM function read variable during serving operation.
				case 0:	//현재동작을 마치지 않아 다음 동작을 준비하고 있지 않을 때 | When you're not ready for your next move.
				serving_move_cnt = EEPROM_read(next_move + 1 + (number * 100));	//다음동작 읽어오기(방향) Read next task (direction)
				run_data_cnt = EEPROM_read(next_move + 51 + (number * 100));	//다음동작 읽어오기(거리) Read next task (distance)
				break;
				case 1:	//현재동작을 마치고 다음동작을 준비할 때 | When you finish your current action and prepare for the next one.
				serving_move_cnt = 0;	//dc모터를 멈춘다. | Stop the DC motor.
				break;
			}
			
			delay_cnt++;	//서빙 동작에 진입 하자마자 다음 동작으로 넘어가지 못하도록 하는 딜레이 변수 1ms 마다 증가. | As soon as the serving motion is entered, the delay variable that prevents moving to the next motion is incremented every 1ms.
			switch(delay_cnt){
				case 0: delay_flag = 0; break;	//0ms일 때 delay_flag 변수값 0 | Delay_flag variable value 0 at 0ms.
				case 50: delay_flag = 1; delay_cnt = 5; break;	//50ms일 때 delay_flag 변수값 1 | delay_flag variable value 1 at 50ms.
			}
			
			
			if((serving_complete == 0 || serving_complete == 2) && delay_flag == 1){	//테이블로 이송 중이거나 이송을 완료하고 돌아오고 있을 때 | When transporting to the table or returning from transport.
				if((left_number > run_data_cnt || right_number > run_data_cnt) && flag == 0){	//이동한 거리가 읽어온 거리보다 값이 클 때 | If the travel distance is greater than the reading distance.
					//flag는 카운터가 한 번만 증가하기 위한 변수 | A flag is a variable whose counter is incremented only once.
					/*로봇이 목표 지점에 도달했을 때 | When the robot reaches the target point*/
					flag = 1;		//flag = 1;
					move_flag = 1;	//move_flag = 1; 다음 동작이 되지 않도록 EEPROM_read 함수를 읽지 않는다. | Do not read the EEPROM_read function to avoid the following behavior:
					int cnt = 0;	//카운터 변수 | counter variable
					cnt++;			//1ms마다 카운터 증가 | The counter is incremented every 1ms.
					
					if(cnt < 50){	//50ms 안에 동작하는 카운터가 한 번만 증가하도록 하는 제어문 | A control statement that causes a counter that operates within 50 ms to increment only once.
						cnt = 0;	//카운터 초기화 | Counter reset
						next_move++;		//다음동작 준비 | Prepare for the next move
						point = 1;			//다음 지점으로 이동(다음동작 준비) | Go to the next point (prepare for the next task)
					}
				}
				
				if(point == 1){		//다음동작을 준비할 때 | when preparing for the next step
					cnt1++;			//cnt1 변수값 1ms마다 증가 | The cnt1 variable value increments every 1 ms.
					switch(cnt1){
						case 0:						//0ms일 때 | at 0ms
						right_number = 0;		//오른쪽 모터 엔코더값을 0으로 초기화 시킨다. | Initialize the right motor encoder value to 0.
						left_number = 0;		//왼쪽 모터 엔코더값을 0으로 초기화 시킨다. | Initialize the left motor encoder value to 0.
						
						break;
						case 500:					//500ms에 도달했을 때 | When 500 ms is reached
						flag = 0;				//flag = 0; (한 번만 증가하기 위한 변수) | flag = 0; (variable is incremented only once)
						cnt1 = 0;				//cnt1을 0으로 초기화 | Initialize cnt1 to 0
						move_flag = 0;			//다음동작을 실행한다. | Execute the following operation.
						point = 0;				//point 0으로 초기화 | reset to point 0
						break;
					}
				}
			}
			switch(serving_move_cnt){	//방향값 | direction value
				case 5:					//서빙동작중 테이블에 도착했을 때 | When you arrive at the table while serving
				move_flag = 1;			//dc모터를 멈춘다. | Stop the dc motor.
				serving_complete = 1;	//테이블에 도착했다고 신호를 보낸다.(1: 테이블 도착, 2: 물건 전달 완료.) | Signals that the table has arrived. (1: table has arrived, 2: goods have been delivered.)
				
				break;
				
				/*서빙 완료 | Serving Complete*/
				case 6:					//동작이 끝났을 때 | when the action is over
				button_led = 1;			//버튼 LED를 켠다. | Turn on the button LED.
				button_led_f = 1;		//버튼 LED를 0.5초마다 blank 시킨다. | The button LED is blanked every 0.5 seconds.
				left_number = 0;		//왼쪽 모터 엔코더값 0으로 초기화 | Reset left motor encoder value to 0
				right_number = 0;		//오른쪽 모터 엔코더값 0으로 초기화 | Reset right motor encoder value to 0
				next_move = 0;			//다음동작 준비 변수 0으로 초기화 | Initialize the next operation preparation variable to 0
				red_led = 0;			//빨간색 LED_OFF | Red LED_OFF
				fnd_com = 0;			//FND blank 안 함 | No FND blank
				flag = 0;				//다음동작 준비 flag 변수 0으로 초기화 | Initialize the next operation preparation flag variable to 0
				set_table_finish = 0;	//테이블 설정 0으로 초기화 | Initialize table setting 0
				delay_flag = 0;			//delay_flag값 0으로 초기화 | Initialize delay_flag to 0
				delay_cnt = 0;			//delay_cnt 0으로 초기화 | delay_cnt initialized to 0
				move_flag = 0;			//eeprom read flag 변수 0으로 초기화 | eeprom read flag variable initialized to 0
				serving_complete = 0;	//서빙동작 단계별 변수 0으로 초기화 | Initialize variable to 0 for each serving operation step
				number = 1;				//테이블 1번으로 초기화 | Initialize to table 1
				serving_move_cnt = 0;	//방향변수 0으로 초기화 | Initialize the direction variable to 0
				run_data_cnt = 0;		//거리변수 0으로 초기화 | Initialize the distance variable to 0
				serving_start = 0;		//서빙동작 종료 | End of serving
				break;
			}
		}
	}
}

/*타이머 카운터0 | timer counter 0*/
ISR(TIMER0_OVF_vect)
{
	TCNT0=0x83;				//타이머 카운터0 1ms | Timer counter 0 1ms
	FND_LED();				//FND_LED 함수 | FND_LED function
	button_control();		//버튼제어 함수 | button control function
	encoder();				//엔코더 함수 | encoder function
	DC_MOTOR();				//DC모터 함수 | DC motor function
	mode_setting_move();	//모드설정 함수 | mode setting function
}

int main(void) // 메인함수 호출 | main function call
{
	DDRA = 0x01; DDRB = 0xff; DDRC = 0x00; DDRD = 0xf8;
	
	button_f = 1;	//전원을 켰을 때 버튼 부분이 동작되지 않도록 하는 부분 | The part that prevents the button part from working when the power is turned on
	
	//UART 설정 통신속도 9600bps | UART setting baud rate 9600bps
	UCSRA=0x00;
	UCSRB=0x18;
	UCSRC=0x06;
	UBRRH=0x00;
	UBRRL=0x33;

	TCCR0=(0<<WGM00) | (0<<COM01) | (0<<COM00) | (0<<WGM01) | (0<<CS02) | (1<<CS01) | (1<<CS00);
	TCNT0=0x83;		//타이머 카운터0 1ms | Timer counter 0 1ms
	OCR0=0x00;
	
	TIMSK = 0x01;	//TIMER_MSK = 0x01;(타이머 카운터0) | TIMER_MSK = 0x01; (timer counter 0)
	
	sei();	//인터럽트 허용 | Interrupt Allowed
	
	while (1)
	{
		/*입력핀 GPIO 변수 | Input Pin GPIO Variables*/
		button = PINC & 0x40;		//버튼은 PINC의 0x40번 핀을 &한 값이다. | The button is the value of &-pin 0x40 of PINC.
		seg_en1 = PINC & 0x01;		//FND 엔코더 1번은 PINC의 0x10번 핀을 &한 값이다. | FND Encoder No. 1 is the value obtained by & subtracting pin 0x10 of PINC.
		seg_en2 = PINC & 0x02;		//FND 엔코더 2번은 PINC의 0x20번 핀을 &한 값이다. | FND Encoder No.2 is the value obtained by subtracting pin 0x20 of PINC.
		right_en1 = PINA & 0x02;	//오른쪽 모터 엔코더 1번은 PINC의 0x10번 핀을 &한 값이다. | Right motor encoder No. 1 is the value of & minus pin 0x10 of PINC.
		right_en2 = PINA & 0x04;	//오른쪽 모터 엔코더 2번은 PINC의 0x20번 핀을 &한 값이다. | The right motor encoder No.2 is the value of & minus pin 0x20 of PINC.
		left_en1 = PINA & 0x08;		//왼쪽 모터 엔코더 1번은 PINC의 0x10번 핀을 &한 값이다. | The left motor encoder No. 1 is the value of & minus pin 0x10 of PINC.
		left_en2 = PINA & 0x10;		//왼쪽 모터 엔코더 1번은 PINC의 0x20번 핀을 &한 값이다. | The left motor encoder No. 1 is the value of & minus pin 0x20 of PINC.
	}
}