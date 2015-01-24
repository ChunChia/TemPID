/*
12 Kanal Temperaturregler
Version f¸r 4 LEDs/Kanal

Der Temperaturregler besitzt 12 Kan‰le. Jeder Kanal liest die Temperatur eines Termocouple F¸hlers ¸ber den ADC ein,
berechnet eine Stellgrˆﬂe mittels eines PID Reglers und ¸bergibt diese an einen Z‰hler, welcher daruas ein PWM Signal erzeugt.
Das PWM Signal steuert ¸ber einen Transistor oder ein SSR (Solid State Relay) die Heizleistung.
Die Temperatur wird von IC (AD595) in einen Spannung (10mV/K) umgestzt. Da hier bis zu 1000∞C erfasst werden sollen und die Referenzspannung des DAC 
5 Volt betr‰gt, wird der Quotient auf 5mV/K eingestellt. Bei einer Auflˆsung von 10 Bit entspricht das LSB also 1000/1024 Grad 
(~1 Grad/ Bit). 
Das Programm besitzt eine Selbstoptimierungsroutine, welche mittels eines Sprunges am PWM Ausgang auf einen fixen Wert, aus dem Temperaturverlauf
die Paramter f¸r den jeweiligen PID Regler berechnet. Die Paramter werden nach den Einstellregeln von Ziegler-Nichols bestimmt. 

Der Regler wird ¸ber die Serielle Schnittstelle gesteuert. baude rate = 9600, data bits = 8, stop bits = 1;
‹bertragungsprotokoll:

Es m¸ssen immer 6 Byts ¸bertragen werden, damit die Daten als g¸ltig gelten. 

1. Byte		Adresse des PID-Reglers, Adresse wird am Ende bei den Interruptsroutinen, serlielle Schnittstelle eingetragen.
2. Byte		Kanalnummer von 1 bis 12
3. Byte		High Byte Sollwert
4. Byte		Low Byte Sollwert (max. Sollwert 1023)
5. Byte		0: Set Temperatur, 1: self optimisation PID Parameter, 3: Aktuelle Temperatur Kanal ch
6. Byte 	Pr¸fsumme, XOR Verkn¸pfung der ersten 5 Byts

WICHTIG: Fuses:	JTAGEN	deaktivieren
				CKDIV8	deaktivieren 

		PortF: PF7..PF4 Pins f¸r das JTAG interface. Wenn enable, pull-up resistors aktiv. Daher JTAG deaktivieren

LEDs:

			SelfOpt
LED1 		Ein, wenn grˆﬂeres DeltaT gemessen
LED2
LED3		nich in verwendung
LED PE2		Zeigt weitern Durchlauf an
*/

#include <C:\WinAVR-20100110\avr\include\stdint.h>			//Standardisierte Datentypen
#include <avr/sleep.h>
//#include <stdlib.h>
//#include <avr/math.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>


volatile uint16_t soll[12];				//Sollwert
volatile uint8_t selfopt[12];			//Selbstoptimierung Ja/Nein, 1/0
volatile uint32_t OCRmax[12];			//OCR Maximalwert setzen
volatile uint32_t OCRset[12];			//OCR fixer Wert
volatile uint8_t TempOut;				//>0 bei Temperaturausgabe
volatile uint8_t SetPointOut;			//>0 bei Setpointausgabe
volatile uint8_t PIDSettingsOut;		//>0 bei PID settings ausgabe
volatile uint8_t PIDOnRatioOut;		    //>0 bei PID on ratio ausgabe
volatile uint8_t ParSetAll;				//1 bei kopieren der PID Paramter von Kanal 1 auf allen anderen 
volatile uint8_t setOCR=0;				//>0 , fixen Wert f¸r Stellwert (OCR) der SSR 
volatile uint8_t holdOCR=0;				//>0 , halte aktuellen OCR Wert
volatile uint8_t ch=0;					//Channel, f¸r COM2
volatile uint8_t tmpHB, tmpLB, tmpUHB, tmpULB; 
volatile uint32_t htmpHB, htmpLB, htmpUHB, htmpULB; 
volatile uint8_t address;			//f¸r COM2
volatile uint8_t n=0;					//Z‰hler f¸r COM2
volatile uint8_t LED=0;					//LED signalisiert korrekt empfangenen Daten an COM2
volatile uint8_t select=0;				//Auswahl an COM2
volatile uint8_t ADCBeendet=0;
volatile uint16_t count_td=0;			//Z‰hlerdruchl‰ufe Timer1, ein Druchlauf = 131ms
volatile uint16_t count_tW=0;			//Z‰hlerdruchl‰ufe Timer1, ein Druchlauf = 131ms
volatile uint16_t count_tLED_COM=0, count_tLEDs;	//Z‰hlerdruchl‰ufe Timer1, ein Druchlauf = 131ms


uint16_t OCR[12]={0x128,0x8A,0xAC,0xA8,0x9A,0x8C,0x88,0xAA,0x9C,0x98,0x12A,0x12C};		//Adressen OC-Register

int32_t Prop[12], Int[12], Diff[12];	//PID Paramter
uint32_t PIDP[12] EEMEM, PIDI[12] EEMEM, PIDD[12] EEMEM;  //EEPROM Speichervariblen der PID Paramter

/*
PortA: 0x22
PortB: 0x25
PortC: 0x28
PortD: 0x2B
PortE: 0x2E
PortF: 0x31
PortG: 0x34
PortH: 0x102
PortJ: 0x105
PortK: 0x108
PortL: 0x10B
*/


struct LED1PB{uint16_t P;				//Portadresse und Pin LED1
			  uint8_t B;}
			  LED1[12]={{0x34,PG4},{0x10b,PL6},{0x2b,PD2},{0x2b,PD6},{0x28,PC0},{0x28,PC4},{0x105,PJ0},{0x105,PJ4},{0x22,PA7},{0x22,PA3},{0x105,PJ7},{0x108,PK4}};

struct LED2PB{uint16_t P;				//Portadresse und Pin LED2
			  uint8_t B;}
			  LED2[12]={{0x10b,PL0},{0x10b,PL7},{0x2B,PD3},{0x2b,PD7},{0x28,PC1},{0x28,PC5},{0x105,PJ1},{0x105,PJ5},{0x22,PA6},{0x22,PA2},{0x108,PK7},{0x34,PG5}};

struct LED3PB{uint16_t P;				//Portadresse und Pin LED3
			  uint8_t B;}
			  LED3[12]={{0x10b,PL2},{0x2B,PD1},{0x2B,PD5},{0x34,PG1},{0x28,PC3},{0x28,PC7},{0x105,PJ3},{0x34,PG2},{0x22,PA4},{0x22,PA0},{0x108,PK5},{0x2e,PE0}};

struct SSRPB {uint16_t P;				//Portadresse und Pin SSR
		      uint8_t B;}
			  SSR[12]={{0x109,PL3},{0x23,PB6},{0x100,PH5},{0x100,PH3},{0x2C,PE4},{0x23,PB7},{0x23,PB5},{0x100,PH4},{0x2C,PE5},{0x2C,PE3},{0x109,PL4},{0x109,PL5}};

struct TCerrPB {uint16_t P;				//Portadress und Pin TCerr
			  uint8_t B;}
			  TCerr[12]={{0x109,PL1},{0x29,PD0},{0x29,PD4},{0x32,PG0},{0x26,PC2},{0x26,PC6},{0x103,PJ2},{0x103,PJ6},{0x20,PA5},{0x20,PA1},{0x106,PK6},{0x2c,PE1}};


uint16_t ist[12];						//IstWert
int16_t err0[12], err1[12], err2[12];	//Fehler
int32_t pid[12];						//Ausgabe PID Regler
uint8_t i, LED_duration;
uint8_t TCok[12];
uint32_t BigLoopCount;

uint32_t ADalt=0, ADneu=0;
int32_t Nenner;
int32_t TempW=0, TempStart=0, delta=0, deltamax=0, k;
uint16_t timeW=0;
uint8_t j=0, first_time=1, count_Wmean=1, selfopt_Start=1;

//Funktionen
uint16_t ADCV(uint8_t i);				//AD Wert abfragen
int32_t div(int32_t divident, int32_t divisor); //Division
int32_t mul(int32_t multiplikand, int32_t multiplikator); //Multiplikation



int main(void)
{

//initalisier Atmel

// Timer/Counter 
TIMSK1 |= (1<<TOIE1); //Interrupt enable, zur Zeiterfassung, 1 Durchgang = 2^10/(8MHz/1024)=131ms
TCCR1A = 0b10101011;  // reset OC at compare match, fast PWM, 10 Bit 
TCCR3A = 0b10101011;  // reset OC at compare match, fast PWM, 10 Bit 
TCCR4A = 0b10101011;  // reset OC at compare match, fast PWM, 10 Bit 
TCCR5A = 0b10101011;  // reset OC at compare match, fast PWM, 10 Bit 

TCCR1B = 0b00001101;  // fast PWM, prescaler 1024, 10 Bit
TCCR3B = 0b00001101;  // fast PWM, prescaler 1024, 10 Bit
TCCR4B = 0b00001101;  // fast PWM, prescaler 1024, 10 Bit
TCCR5B = 0b00001101;  // fast PWM, prescaler 1024, 10 Bit

//Timer0 f¸r R¸cksetzten des Byte-Z‰hlers der COM2
TCCR0A |= (0<<WGM01);					//normal mode
TCCR0B |= (1<<CS02)|(1<<CS00);			//prescaler 1024
TIMSK0 |= (1<<TOIE0);					//Interrupt enable, Setzt Byte-Z‰hler auf 0
//OCR0A = 249;							//1/(8MHz/1024*250) = 32ms

/*
//Timer2 f¸r Zeitverzˆgerung beim selfopt, freilaufend
TCCR2A |= (1<<WGM21);					//CTC mode
TCCR2B |= (1<<CS22)|(1<<CS21)|(1<<CS20);//prescaler 1024
TIMSK2 |= (1<<OCIE2A);					//Interrupt enable, z‰hlt Durchl‰ufe des Z‰hlers
OCR2A = 249;							//1/(8MHz/1024*250) = 32ms
*/

//Portsettings, f¸r LED Test alle Error Eing‰nge als Ausg‰nge setzen, SSR Ausg‰nge m¸ssen auch als Ausgang gesetzt werden
DDRA = 0xFF; 						//Pins f¸r Ein- und Ausgabe
DDRB = 0b11100000;
DDRC = 0xFF;
DDRD = 0xFF;
DDRE = 0b00111111;
DDRG = 0b00110111;
DDRH = 0b00111000;
DDRJ = 0xFF;
DDRK = 0xF0;
DDRL = 0xFF;					


//Setze Z‰hlerregister so, dass die Relays relativ zum Netzt zu unterschiedlichen Zeiten schalten
TCNT1=0;
TCNT3=256;
TCNT4=512;
TCNT5=768;

// ADC,  	// externe Refernz, right adjust, single conversion
ADCSRA = (1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);  //ADC enable, ADC Interrupt enabel, Prescaler 128
DIDR0 = 0xff; 							// disable digital Input of ADC7:0
DIDR2 = 0b10111111;						// disable digital Input of ADC15:8, enable digital input ADC14

// sleep mode 
//SMCR = (1<<SM0)|(1<<SE);  			// ADC Noise reduction, nicht mˆglich, weil im sleep modus die COM2 den uC nicht aufwecken kann  
//set_sleep_mode(SLEEP_MODE_ADC);			//ADC Noise reduction mode ausgew‰hlt, in der Funktion ADCV wird mit sleep_mode() eine Wandlung und sleep gestartet, Kann zu Problemen mit COM verursachen => wechsle sleep mode
ACSR |= (1<<ACD);  						//disable Analog Compertor, Strom sparen


/* Com 2 */
/*  frame format: 8data, 1 stop bit */
/* Set baud rate */
UBRR2 = 51;  /* baud rate = 9600 */
/* Enable receiver (and transmitter) */
UCSR2B = (1<<RXCIE2)|(1<<RXEN2)|(1<<TXEN2);  // enable Interrupt


//Watchdog
//WDTCSR |= (1<<WDIE);
//wdt_enable(WDTO_1S);



////////////////////////////////////////////////////////////////////////////////////////
//Main

sei();									//enable Interrupts
	

// Startprozedur Anfang
//LED Test

PORTA = 0b11011101;						
PORTC = 0b10111011;
PORTD = 0b11101110;
PORTE = 0b00000101;
PORTG = 0b00110110;
PORTJ = 0b10111011;
PORTK = 0b10110000;
PORTL = 0b11000101;

count_td=0;
while (count_td<10);

PORTA = 0;
PORTC = 0;
PORTD = 0;
PORTE = 0;
PORTG = 0;
PORTJ = 0;
PORTK = 0;
PORTL = 0;

//Portsetting after LED Test, Error als Eingang
DDRA = 0b11011101; 						//Pins f¸r Ein- und Ausgabe
DDRC = 0b10111011;
DDRD = 0b11101110;
DDRE = 0b00111101;
DDRG = 0b00110110;
DDRJ = 0b10111011;
DDRK = 0b10110000;
DDRL = 0b11111101;


for (i=0;i<12;i++)						//Initalisieren 
	{
	ist[i]=0;
	soll[i]=ADCV(i);
	Prop[i]= eeprom_read_dword(&PIDP[i]);
	Int[i]= eeprom_read_dword(&PIDI[i]);
	Diff[i]= eeprom_read_dword(&PIDD[i]);
	err0[i]=0;
	err1[i]=0;
	err2[i]=0;
	pid[i]=0;
	selfopt[i]=0;
	}

// Startprozedur Ende

//selfopt[0]=1;

BigLoopCount=0;
while (1)								//loop
{
	BigLoopCount++;
	if (BigLoopCount>100) BigLoopCount=0;

	if (LED) 										//LED wird von COM2 auf 1 gesetzt
		if (count_tLED_COM < LED_duration)			//Flash LED, wenn COM2 g¸ltige Daten empfangen hat
			if (count_tLED_COM & 0x01) PORTE |= (1<<PE2);
			else PORTE &= 0xFF ^ (1<<PE2);			//Led ausschalten
		else
		{
			LED = 0;
			PORTE &= 0xFF ^ (1<<PE2);			//Led ausschalten
		}			
	else	
		count_tLED_COM = 0;

	for (i=0;i<12;i++) 
	{
		TCok[i]=(_MMIO_BYTE(TCerr[i].P) & (1<<TCerr[i].B));		//Einlesen von TC-Errors
	}

	for (i=0;i<12;i++) 
	if (TCok[i]) 
	{
		//if (_MMIO_BYTE(SSR[i].P) & (1<<(SSR[i].B)) && (pid[i]>0))					//LED3 (blau) ein, wenn SSR ein
		if (pid[i]>(BigLoopCount*10000))											//LED3 (blau) proportional zu PWM SSRs
			_MMIO_BYTE(LED3[i].P) |= (1<<LED3[i].B);
		else
			_MMIO_BYTE(LED3[i].P) &= ~(1<<LED3[i].B);
	}
	
	for (i=0;i<12;i++)
	{
		ist[i] = ADCV(i);
		if (TCok[i] && (ist[i] < 700) && ((PORTB & (1<<PB4))==0)) //Fehler und Tmax abfragen, 0 Fehler, TC nicht angeschlossen oder unterborchen
		{
			if (selfopt[i])						//Selbstoptimierung ausf¸hren
			{
				if (selfopt_Start)
				{
					ADalt =  ist[i];			//
					cli();
					count_tW = 0;				//Starte Zeitz‰hler	f¸r Zeit Wendepunkt
					sei();
					TempStart = ADalt;			//Starttemperatur
					_MMIO_BYTE(LED1[i].P) |= (1<<LED1[i].B);		//LED1 (gelb) ein
					_MMIO_BYTE(LED2[i].P) &= ~(1<<LED2[i].B);		//LED2 (gr¸n) auschalten
					selfopt_Start = 0;
				} //End selfopt_Start

				_SFR_MEM16(OCR[i]) = 1023;			//Sprung, PID wird auf Maximum gesetzt
					
				if (j<5)
				{
					if ((ADneu=ist[i]) > ADalt)
					{
						if (first_time)				//erster Durchlauf liefert keine richtigen Wert (count_td starte zuf‰llig zwischen zwei Temp-Abfragen)
						{
							ADalt = ADneu;
							cli();
							count_td = 1;			//Starte Z‰hler f¸r Zeitdiff. zwischen zwei Temp-Abfragen f¸r Ableitung
							sei();
							deltamax = 0;
							first_time = 0;
						}
						else
						{	
							cli();
							delta = div(mul((ADneu-ADalt),10000), count_td);    //Delta in mV mal 10 (x10000)
							sei();
							if (delta >= deltamax)
							{
								_MMIO_BYTE(LED1[i].P) |= (1<<LED1[i].B);		//LED1 (gelb) ein
								_MMIO_BYTE(LED2[i].P) &= ~(1<<LED2[i].B);		//LED2 (gr¸n) wieder auschalten

								if (delta==deltamax) 				//Mittelwertbildung der Wendepunkt-Zeit und -Temp. bei gleichen Delta
								{
									cli();
									timeW = timeW+count_tW;
									sei();
									TempW = TempW+ADalt;
									++count_Wmean;					//Anzahl Punkt mit gleichem Delta
								} 
								else 
								{
									count_Wmean=1;					//Keine Punkt mit gleichem Dealt => nur ein Wert f¸r Mittelwertbildung
									cli();
									timeW = count_tW;				//Festhalten der Wendepunktzeit	
									sei();
									TempW = ADalt;					//Festhalten der Wendepunktemp
									deltamax = delta;
								}
								PINE |= (1<<PE2);					//LED_COM toggeld bei Auftreden eines deltamax
								j=0;
							}
							else //(delta >= deltamax)
							{
								j++;
								_MMIO_BYTE(LED2[i].P) |= (1<<LED2[i].B);		//LED2 (gr¸n) einschalten
							} //end else (delta >= deltamax)
							
							ADalt = ADneu;
							cli();
							count_td=1;							//Zeit f¸r Zeitdiff neu starten
							sei();
						}	//end else first_time
					}	//end (ADneu>ADalt)
				}		//end if (j < 5)
				else
				{
					timeW = div(div(mul(timeW,131),count_Wmean), 1000);		//Zeit Wendepunkt in Sekunden, ein Z‰hlerdurchlauf = 131 ms
					TempW = div(mul(TempW,1000),count_Wmean);				//Temperatur Wendepunkt x 1000
					k = div(mul(deltamax,100),131);							//Steigung x 1000 (deltamx bereits mal 10)
					Nenner = mul(k,timeW)+mul(TempStart,1000)-TempW;		//mul(k,timeW)+mul(TempStart,1000)-TempW
					if (Nenner<0) Nenner = -Nenner;
					Prop[i] = div(1200000,Nenner); 							//Werte mal 1000
					Int[i] = div(mul(600000,k),mul(Nenner,Nenner));
					Diff[i] = div(600000, k);
			
					//Speichern in EEPROM
					eeprom_write_dword(&PIDP[i], Prop[i]);
					eeprom_write_dword(&PIDI[i], Int[i]);
					eeprom_write_dword(&PIDD[i], Diff[i]);
					

					PORTE &= ~(1<<PE2);					//LED abschalten, welche eventuell durch selfopt eingeschalten wurde
					selfopt_Start = 1;
					selfopt[i] = 0;
					first_time = 1;
					j = 0;
				}	//end else (j < 5)
			}


			else  //keine Selpstopt., PID Teil
			{
				
				if ((Int[i] == 0) || (Int[i] == 0xFFFFFFFF))	//PID nur aktiv, wenn zuvor eine Selbstoptimierung durchlaufen wurde (Integraleineteil nicht Null und nicht 0xFFFFFFFF)
				{				    		
					if (count_tLEDs < 5)
					{
						_MMIO_BYTE(LED2[i].P) |= (1<<LED2[i].B);
						_MMIO_BYTE(LED1[i].P) &= ~(1<<LED1[i].B);
					}
					else
					{	
						if (count_tLEDs < 10)
						{
							_MMIO_BYTE(LED2[i].P) &= ~(1<<LED2[i].B);
							_MMIO_BYTE(LED1[i].P) |= (1<<(LED1[i].B));
						}
						else
							count_tLEDs = 0;
					}
				} //end if ((Int[i] == 0) || (Int[i] == 0xFFFFFFFF))
				else
				{
					err2[i]=err1[i];
					err1[i]=err0[i];
					cli();								//Interrupts disablen, Zugriff auch 16 Bit Register in Interruptroutine(soll)
					err0[i]=soll[i]-ist[i];
					sei();

					pid[i] = pid[i] + mul(Prop[i],(err0[i] - err1[i]))
									+ mul(Int[i],err0[i])
									+ mul(Diff[i],(err0[i] - mul(2,err1[i]) + err2[i]));


					if (pid[i]<0)
						pid[i]=0;
					if (pid[i]>=1023000)							//pid begrenzen 
						pid[i]=1023000;
					cli();
					if (pid[i]>OCRmax[i])							//pid begrenzen 
						pid[i]=OCRmax[i];
					sei();


						_SFR_MEM16(OCR[i])= div(pid[i],1000);		//schreiben in OCRx, dividiere 

					if ((err0[i]<2) && (soll[i]>0))					//((err0[i]<2) && ((_MMIO_BYTE(SSR[i].P) & (1<<(SSR[i].B))) && (pid[i]>0)))	//LED2 ein, wenn Fehler <2 
						_MMIO_BYTE(LED2[i].P) |= (1<<LED2[i].B);
					else
						_MMIO_BYTE(LED2[i].P) &= ~(1<<LED2[i].B);

					if ((err0[i]>-2) && (soll[i]>0))				//((err0[i]>-2) && ((_MMIO_BYTE(SSR[i].P) & (1<<(SSR[i].B))) && (pid[i]>0)))	//LED1 ein, wenn Fehler > -2
						_MMIO_BYTE(LED1[i].P) |= (1<<(LED1[i].B));
					else
						_MMIO_BYTE(LED1[i].P) &= ~(1<<LED1[i].B);


					/*if ((err0[i]<2) && (err0[i]>-2) && (soll[i]>0))					//((err0[i]<2) && (err0[i]>-2) && ((_MMIO_BYTE(SSR[i].P) & (1<<(SSR[i].B))) && (pid[i]>0)))	//LED2 ein, wenn Fehler <+/-2 
						_MMIO_BYTE(LED2[i].P) |= (1<<LED2[i].B);
					else
						_MMIO_BYTE(LED2[i].P) &= ~(1<<LED2[i].B);*/
						

				}//end else
			}   //end else
		}		//end if ((_MMIO_BYTE(TCerr[i].P) & (1<<TCerr[i].B)) && (ADCV(i) < 700))
		else
		{	
			_SFR_MEM16(OCR[i]) = 0;			//Bei TC Fehler oder Temp > 700∞C, Ausgang Null setzen
			_MMIO_BYTE(LED1[i].P) &= ~(1<<LED1[i].B);		//LED1 aus
			_MMIO_BYTE(LED2[i].P) &= ~(1<<LED2[i].B);		//LED2 aus
			_MMIO_BYTE(LED3[i].P) &= ~(1<<LED3[i].B);		//LED3 aus
		}
		

		//Asugabe an COM2
		if ((TempOut > 0) && (TempOut <= 12))	//Temperaturausgabe
		{
			
		//	ist[i]=ADCV(i);						//read value from ADC
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe Kanal
						UDR2 = i+1;
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe Temperatur
						UDR2 = (ist[i]>>8);
			while ( !( UCSR2A & (1<<UDRE2)) );
						UDR2 = ist[i];
			
			++TempOut;
			if (TempOut > 12)
				TempOut = 0;
		}

		if ((SetPointOut > 0) && (SetPointOut <= 12))			//Sollwert Ausgabe
		{
						
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe Kanal
						UDR2 = i+1;
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe SetPoint
						UDR2 = (soll[i]>>8);
			while ( !( UCSR2A & (1<<UDRE2)) );
						UDR2 = soll[i];
			
			++SetPointOut;
			if (SetPointOut > 12)
				SetPointOut = 0;
		}


		if ((PIDOnRatioOut > 0) && (PIDOnRatioOut <= 12))			//PID On Ratio abfrage
		{
						
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe Kanal
						UDR2 = i+1;
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe PIDOnRatio
						UDR2 = (pid[i]) & 0xFF;
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe PIDOnRatio
						UDR2 = (pid[i]>>8) & 0xFF;
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe PIDOnRatio
						UDR2 = (pid[i]>>16) & 0xFF;			
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe PIDOnRatio
						UDR2 = (pid[i]>>24) & 0xFF;
			
			++PIDOnRatioOut;
			if (PIDOnRatioOut > 12)
				PIDOnRatioOut = 0;
		}				

		if ((PIDSettingsOut > 0) && (PIDSettingsOut <= 12))			//PID setpoint abfrage
		{
						
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe Kanal
						UDR2 = i+1;
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe SetPoint
						UDR2 = (Prop[i]) & 0xFF;
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe SetPoint
						UDR2 = (Prop[i]>>8) & 0xFF;
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe SetPoint
						UDR2 = (Prop[i]>>16) & 0xFF;			
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe SetPoint
						UDR2 = (Prop[i]>>24) & 0xFF;			
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe SetPoint
						UDR2 = (Int[i]) & 0xFF;
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe SetPoint
						UDR2 = (Int[i]>>8) & 0xFF;
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe SetPoint
						UDR2 = (Int[i]>>16) & 0xFF;			
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe SetPoint
						UDR2 = (Int[i]>>24) & 0xFF;			
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe SetPoint
						UDR2 = (Diff[i]) & 0xFF;
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe SetPoint
						UDR2 = (Diff[i]>>8) & 0xFF;
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe SetPoint
						UDR2 = (Diff[i]>>16) & 0xFF;			
			while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe SetPoint
						UDR2 = (Diff[i]>>24) & 0xFF;			
					
			++PIDSettingsOut;
			if (PIDSettingsOut > 12)
				PIDSettingsOut = 0;
		}
		
		if ((ParSetAll > 0) && (ParSetAll <= 12))	// ‹bertragen der Regelparameter von Kanal auf alle anderen
		{
			Prop[i] = Prop[0];
			Int[i] = Int[0];
			Diff[i] = Diff[0];
			//speichern ins EEPROM
			eeprom_write_dword(&PIDP[i], Prop[0]);
			eeprom_write_dword(&PIDI[i], Int[0]);
			eeprom_write_dword(&PIDD[i], Diff[0]);
				
			++ParSetAll;
		}
			if (ParSetAll > 12)
				ParSetAll = 0;
		
			
	}	//end for, next i

//	wdt_reset();
}	// end while
return (0);
}





//////////////////////////////////////////////////////////////////////////////////////////
//Funktionen

int32_t mul(int32_t multiplikand, int32_t multiplikator)
{
	int32_t produkt=0;
	int8_t m, sign=0;
	if ((multiplikand ^ multiplikator)>>31) sign = 1;
	if (multiplikand < 0) multiplikand = -multiplikand;
	if (multiplikator < 0) multiplikator = -multiplikator;
	for (m=14; m>=0; m--)
	{
		produkt = (produkt<<1);
		if ((multiplikator & (1<<m))) produkt = produkt+multiplikand;
	}
	if (sign) produkt = -produkt;
	return produkt;
}


///////////



int32_t div(int32_t divident, int32_t divisor)
{
	int32_t quotient = 0, rest;
	int8_t sign=0;
	
	if (divisor == 0)
		quotient = divident;
	else
	{
		if ((divident ^ divisor)>>31) sign = 1;
		if (divident<0) divident = -divident;
		if (divisor<0) divisor = -divisor;

		while( divident >= divisor )
			{
			divident -= divisor;
			quotient++;
			}
		rest = divident;
		if (sign) quotient = -quotient;
	}
	return quotient;
}
///////////



uint16_t ADCV(uint8_t i)
{   
uint8_t j;
uint16_t ADCm=0;
	if (i<8)							//Setze Input Kanal
		{
		ADMUX = i;
		ADCSRB &= ~(1<<MUX5);
		}
	else
		{
		ADMUX = i-8;
		ADCSRB |= (1<<MUX5);
		}
	for (j=0; j<10; j++) 				//Mittelung ¸ber x Werte
	{
		//sleep_mode();					//startet AD-Wandlung, wenn set_sleep_mode = ADC ist
		ADCSRA |= (1<<ADSC);			//starte AD-Wandlung
		while (!ADCBeendet);			//warte auf ADC
		ADCBeendet=0;
		ADCm += ADC; 
	}
	return ADCm/10;
}

///////////////


// Interruptsroutinen

ISR(USART2_RX_vect)						//Serielle Schnittstelle abfragen
{ 
	uint8_t Pruefsumme=0, m;
	uint16_t tmp_soll = 0;
	++n;

	switch (n)
	{
		case 1:
			address = UDR2;						//Board Nr. einlesen			
			TCNT0 = 0;							//R¸cksetzten Timer, bei Overflow (zu wenig Byte empfangen) wird n=0 gesetzt
			break;
		case 2:
			ch = UDR2;							//Channel Nr. einlesen
			TCNT0 = 0;							//R¸cksetzten Timer, bei Overflow (zu wenig Byte empfangen) wird n=0 gesetzt	
			break;
		case 3:
			tmpUHB = UDR2;						//highbyte of high word von soll einlesen 
			htmpUHB=tmpUHB;
			TCNT0 = 0;							//R¸cksetzten Timer, bei Overflow (zu wenig Byte empfangen) wird n=0 gesetzt	
			break;
		case 4:
			tmpULB = UDR2;						//lowbyte of high word einlesen 
			htmpULB = tmpULB;
			TCNT0 = 0;							//R¸cksetzten Timer, bei Overflow (zu wenig Byte empfangen) wird n=0 gesetzt	
			break;
		case 5:
			tmpHB = UDR2;						//highbyte of low word von soll einlesen 
			htmpHB = tmpHB;
			TCNT0 = 0;							//R¸cksetzten Timer, bei Overflow (zu wenig Byte empfangen) wird n=0 gesetzt	
			break;
		case 6:
			tmpLB = UDR2;						//lowbyte of low word einlesen 
			htmpLB = tmpLB;
			TCNT0 = 0;							//R¸cksetzten Timer, bei Overflow (zu wenig Byte empfangen) wird n=0 gesetzt	
			break;
		case 7:
			select = UDR2; 						//0: Temperatur setzen 1: selfopt, 2: Temperaturausgabe, 3: Copy CH1 PID settings to all ch, 
												//4: Setpoint ausgeben, 5: PID Settings ausgeben, 6: set P, 7: set I, 8: set D, 
												//9: write PID settings to EEPROM
			TCNT0 = 0;							//R¸cksetzten Timer, bei Overflow (zu wenig Byte empfangen) wird n=0 gesetzt				
			break;
		case 8:
			Pruefsumme = UDR2;
			
			if ((Pruefsumme == (address ^ ch ^ tmpHB ^ tmpLB  ^ tmpUHB ^ tmpULB ^ select)) && (ch > 0) && (ch<=12) && (address == 1))//XOR Verkn¸pfung und Adressabfrage
			{
				--ch;
				if (select==0) 
				{
					tmp_soll = (tmpHB<<8)+tmpLB;	
					if (tmp_soll >=	0) soll[ch] = tmp_soll;		//Set Soll-Temp
				}
				if (select==1) 									//Starte selfopt
				{
					m=0;					
					while (!(selfopt[m]) && (m < 11))			//¸berpr¸fen ob nicht schon eine anders selfopt gesetzt, darf nicht sein
						m++;
					if (m == 11) selfopt[ch] = 1;
				}
				if (select == 2) TempOut = 1;					//gib Temperatur aus

				if (select == 3) ParSetAll = 1;					//¸betrage PID Paramter von Kanal 1 auf alle anderen

				if (select == 4) SetPointOut = 1;				//gib Setpoint aus

				if (select == 5) PIDSettingsOut = 1;			//gib PID settings aus

				if (select == 6) 								//set P
				{
					Prop[ch] = (htmpUHB<<24) + (htmpULB<<16) + (htmpHB<<8) + htmpLB;
				}

				if (select == 7) 								//set I
				{
					Int[ch] = (htmpUHB<<24) + (htmpULB<<16) + (htmpHB<<8) + htmpLB;
				}

				if (select == 8) 								//set D
				{
					Diff[ch] = (htmpUHB<<24) + (htmpULB<<16) + (htmpHB<<8) + htmpLB;
				}
				
				if (select == 9) 								//write EEPROM
				{	
					//speichern ins EEPROM
					eeprom_write_dword(&PIDP[ch], Prop[ch]);
					eeprom_write_dword(&PIDI[ch], Int[ch]);
					eeprom_write_dword(&PIDD[ch], Diff[ch]);
				}

				if (select == 10) PIDOnRatioOut = 1;			//gib PID On ratio aus


				if (select == 11) 								//set 
				{
					//pid[ch] = (htmpUHB<<24) + (htmpULB<<16) + (htmpHB<<8) + htmpLB;
				}
				if (select== 12)
				{
					OCRmax[ch] = ((htmpHB<<8) + htmpLB);			//Setze OCR Maximalwert, 16 Bit Wert
					OCRmax[ch] = mul(OCRmax[ch],1000);
				}

				if (select==13) holdOCR = 1;						// h‰lt OCR auf aktuellen Wert

				if (select==14) setOCR = 1;							//aktiviert fixen Wert f¸r Stellwert (OCR) der SSR

				if (select==15)
				{
					OCRset[ch] = ((htmpHB<<8) + htmpLB);			//Setze OCR auf fixen Wert, 16 Bit Wert
				}
				LED_duration = mul((select+1),3); 
				LED=1;											//Setzt StatusLED auf 1, LED leuchtet kurz auf bei g¸ltigen Datensatz
				
				while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe der empf. Bytes an COM
					UDR2 = n;
			}
			else 
			{
				while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe Fehler an COM
					UDR2 = 0xFF;
			}
			n=0;
			break;
	}
}


////////////////////////////////////////////////////////////////////
//Interrupts

ISR(TIMER0_OVF_vect)
{
	while ( !( UCSR2A & (1<<UDRE2)) );	//Ausgabe Kanal
		UDR2 = n;						//schreibt bei Zeit¸berschreitung von empf. Daten Anzahl der empf. Bytes zur¸ck

	n=0;
}

ISR(TIMER1_OVF_vect)
{
++count_td;
++count_tW;
++count_tLED_COM;
++count_tLEDs;
}

ISR(WDT_vect)
{
}

ISR(ADC_vect)
{
ADCBeendet=1;
}
