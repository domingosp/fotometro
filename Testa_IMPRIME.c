/*
===============================================================================
Name        : Liga_led.c
Author      : $(author)
Version     :
Copyright   : $(copyright)
Description : main definition
===============================================================================
*/

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>
#include <NXP/crp.h>
//#include "C:\Users\DOMINGOS\Documents\MCUXpressoIDE_10.2.1_795\workspace\LIB_LPC1700_USUARIO\inc\UART0_Lib.h"
#include <stdio.h>

//__CRP const unsigned int CRP_WORD = CRP_NO_CRP;
extern int __sys_write (int iFileHandle, char *pcBuffer, int iLength);

//----------DEFINIÇÕES-----------------//

#define I2EN 1<<6
#define STA 1<<5
#define STO 1<<4
#define SI 1<<3
#define AA 1<<2

#define ACK 1<<2
#define NACK 0<<2

#define OP_ESCRITA 0
#define OP_LEITURA 1
//---------DADOS PARA O SENSOR---------------------
#define PowerOn 0b00000001
//#define add "0b01000011"  //(endereço do sensor)
#define PowerDown 0b00000000
#define Reset 0b00000111
#define MeasureMode 0b00100000//(MODO DE MEDIÇÃO - One time HIGH)


//---------- Protótipos de Função ---------------//
void Config_UART0(); 				//declara função que configura UART0
void Config_PWM1();					//declara função que configura os PWM's módulo 1
void Config_ADC();					//declara função que configura ADC
void funcao_ADC3();                 //declara função que imprime o valor do canal analógico 3
void funcao_ADC0();                 //declara função que imprime o valor do canal analógico 0
void Config_I2C1();                 //declara função que configura I2C[1]
//------------Funções para leitura do sensor digital------------------//
void COND_STOP();
unsigned char I2C_ENVIA_DADO(unsigned char dado_i2c);
unsigned char I2C_RECEBE_DADO(unsigned char *dado_i2c, unsigned char acknowledge);
unsigned char I2C_ADD_MEMORIA(unsigned char add, unsigned char operacao);
unsigned char I2C_ESCREVE(unsigned char add, unsigned char dado);
unsigned char I2C_LE(unsigned char add, unsigned char *dado_rec, unsigned char *dado_rec2);


//---------- Variáveis GLOBAIS ---------------//
unsigned char Buffer_RX[]= {"     "}; //Buffer para recepção de dados
int i=0;                              //variavel i para recepção de dados
unsigned int medida;
unsigned char dado_rec;
unsigned char dado_rec2;

//variaveis de controle do DC do PWM:
unsigned char pwm_1 = 0, pwm_2 = 0, pwm_3 = 0;

//-----------------Função interrupção da UART0-----------//
void UART0_IRQHandler()
{

if(i<=4) //enquanto i for menor que 4 fica dentro
{
		Buffer_RX[i]= LPC_UART0->RBR; //Buffer[i] rececebe o respectivo dado que esta na UART0
		switch(Buffer_RX[0])
		{
    	    case 'A': //Se for A executa as ações do canal ADC[3]
    	    	funcao_ADC3();
    	    	i=0;                     //Reinicia o i

    	    	break;

    	    case 'B': //Se for B executa as ações do canal ADC[0]
    	    	funcao_ADC0();
    	    	i=0; //Reinicia o i

    	    	break;

    	    case 'C': //Se for C executa as ações do canal I2C[1]

    	    	I2C_ESCREVE(0b0100011, MeasureMode); //Envia o comando para realizar a medida
    	     	I2C_LE(0b0100011, &dado_rec, &dado_rec2); //Lê o valor medido
    	    	//Depois da leitura o sensor deve executar automaticamente o PowerDown
    	    	medida = (dado_rec<<8)|dado_rec2;
    	    	printf("%ld", medida);

    	     	i=0; //Reinicia o i

    	    	break;

    	    case 'P':                     //Se for P altera os devidos valores do PWM
    	    	i++;  					  //Incrementa o i

    	    	if(i==5) //Quando o i chegar em 5, é pq acabou os dados de PWM, então ele entra
    	    	{
    	    		//Switch case para saber QUAL PWM é
    	    		switch(Buffer_RX[1])
    	    	    {
    	    		case '1':               //Caso for pwm 1
    	    			pwm_1 =100*(Buffer_RX[2]-'0')+ 10*(Buffer_RX[3]-'0')+(Buffer_RX[4]-'0');
    	    			LPC_PWM1->MR1 = pwm_1;
    	    			printf("%03d",pwm_1);
    	    			i=0;
    	    			LPC_PWM1->LER = 1<<1;
    	    	    break;

    	    		case '2':               //Caso for pwm 2
    	    			pwm_2 =100*(Buffer_RX[2]-'0')+ 10*(Buffer_RX[3]-'0')+(Buffer_RX[4]-'0');
    	    			LPC_PWM1->MR2 = pwm_2;
    	    			printf("%03d",pwm_2);
    	    			i=0;
    	    			LPC_PWM1->LER = 1<<2;
    	    	    break;

    	    	    case '3':               //Caso for pwm 3
    	    	    	pwm_3 =100*(Buffer_RX[2]-'0')+ 10*(Buffer_RX[3]-'0')+(Buffer_RX[4]-'0');
    	    	    	LPC_PWM1->MR3 = pwm_3;
    	    	    	printf("%03d",pwm_3);
    	    	    	i=0;
    	    	    	LPC_PWM1->LER = 1<<1;
    	    	    break;
    	    	    }
    	    	}
    	    	 break;

			}

}//fecha if(i<=4)

}//end função interrupção UART0


//-----------------------Função Principal-----------//
int main()
{
    SystemInit(); 						//Configura os recursos básicos do Microcontrolador ->Clk = 100 MHz
    Config_UART0(); 					//Chama a função que configura UART0
    Config_PWM1();	     				//Chama função que configura o pwm
    Config_ADC();                       //Chama função que configura o canal ADC
    Config_I2C1();                      //Chama a função que configura a interface I2C

    while(1){}                      //Loop Infinito que não faz nada
    return 0 ;

}//end int main


void Config_UART0()
{
	LPC_SC->PCONP = 1<<3; // Ativa UART0
	Config_UART0_RS232(19200);

	LPC_UART0->IER |= 1; //Habilita a interrupção RBR

	NVIC_SetPriority(UART0_IRQn,0); //Nível de prioridade da interrupção é 0.
	NVIC_EnableIRQ(UART0_IRQn);
}


void Config_PWM1()
{
	LPC_SC->PCONP |= (1<<6); //PWM1 = ENABLE

	//C_CLK = 100 MHz
	//PCLK_PWM1 = C_CLK/1 = 100 MHz
	LPC_SC->PCLKSEL0 = ((LPC_SC->PCLKSEL0&~(0b11<<12))|(1<<12));

	//Configura os respectivos pinos para os 3 pwm's
	LPC_PINCON->PINSEL3 = ((LPC_PINCON->PINSEL3&~(0b11<<4))|(2<<4)); //P1.18 = PWM1[1]
	LPC_PINCON->PINSEL3 = ((LPC_PINCON->PINSEL3&~(0b11<<8))|(2<<8)); //P1.20 = PWM1[2]
	LPC_PINCON->PINSEL3 = ((LPC_PINCON->PINSEL3&~(0b11<<10))|(2<<10)); //P1.21 = PWM1[3]

	//Prescale Counter incrementado na borda de subida do sinal de clock
	LPC_PWM1->CTCR &= ~(0b11<<0);

	//---Não mexer nessa região
	LPC_PWM1->PR = 99; 			//Fator de divisão --> 100
	LPC_PWM1->MR0 = 255;		//Perídodo do sinal PWM --> T=((99+1)*255)/100Mhz = 255 us
	//-----

	LPC_PWM1->MR1 = pwm_1;       //Inicia o periodo com o valor do pwm_1
	LPC_PWM1->MR2 = pwm_2;		//Inicia o periodo com o valor do pwm_2
	LPC_PWM1->MR3 =	pwm_3;		//Inicia o periodo com o valor do pwm_3

	LPC_PWM1->MCR = 1<<1;			//Reinicia os contadores, caso ocorra um evento de comparação no canal 0
	LPC_PWM1->LER = 0b1111;			//Habilita a atualização dos shadows registers do 0 ao 3

	//Habilita as 3 saídas de PWM
	LPC_PWM1->PCR = 1<<9|1<<10|1<<11;

	//Módulo pwm--> Habilita os contadores
	LPC_PWM1->TCR = 1|1<<3;

}

void Config_ADC()
{
	LPC_SC->PCONP |= 1<<12; 			//Ativa o módulo ADC

	LPC_SC->PCLKSEL0 &= ~(0b11<<24);	//ClckADC= c_clck/4 = 25MHz

	//-------------------AD0[3]---------------------------------------------------------------------
	//Configura o Pino P0.26 para função AD0[3]
	LPC_PINCON->PINSEL1 = ((LPC_PINCON->PINSEL1&~(0b11<<20))|(0b01<<20)); //Entrada do LDR

	//Desabilita os resistores de pull-up e pull-down do pino P0.26 (AD0[3])
	LPC_PINCON->PINMODE1 = ((LPC_PINCON->PINMODE1&~(0b11<<20))|(0b10<<20));


    //------------------AD0[0]----------------------------------------------------------------------
    //Configura o pino P0.23 para a função AD0[0]
    LPC_PINCON->PINSEL1 = ((LPC_PINCON->PINSEL1&~(0b11<<14))|(0b01<<14)); //Entrada do TEMT6000

    //Desabilita os resistores de pull-up e pull-down do pino P0.23 (AD0[0])
    LPC_PINCON->PINMODE1 = ((LPC_PINCON->PINMODE1&~(0b11<<14))|(0b10<<14));


    LPC_ADC->ADCR |= 0b000<<24; //Limpa o bit START

    ///PCLK_ADC = 25MHz
    //CLKDIV = 1: Freq ADC = PCLK_ADC/(CLKDIV+1) = 12.5 MHz  (1<<8)
    //BURST = 0: Modo de conversão BURST                     (0<<16)
    //Canal Analógico 0: pinos AD0[0]                        (1<<0)
    //PDN = 1: Modo operacional (PDN = 0: Modo power-down)   (1<<21)
    //START = 001: Inicia a conversão no modo normal
    LPC_ADC->ADCR = 1<<0|1<<8|0<<16|1<<21|0b001<<24;
    //LPC_ADC->ADCR = 1<<3|1<<8|0<<16|1<<21|0b001<<24;

}

void funcao_ADC3()
{

	while(!(LPC_ADC->ADDR3&(1<<31)));
	printf("%04ld", ((long)((LPC_ADC->ADDR3&(0xFFF<<4))>>4)*3300)/4096); //Imprime leitura do LDR
	LPC_ADC->ADCR |= 0b000<<24;	   //Para a conversão

}

void funcao_ADC0()
{

	while(!(LPC_ADC->ADDR0&(1<<31)));
	printf("%04ld", ((long)((LPC_ADC->ADDR0&(0xFFF<<4))>>4)*3300)/4096); //Imprime a leitura do TEMT6000
	LPC_ADC->ADCR |= 0b000<<24;	   //Para a conversão

}

void Config_I2C1()
{
	LPC_SC->PCONP |= 1<<19; // Ativa o módulo I2C1
	LPC_SC->PCLKSEL1 = ((LPC_SC->PCLKSEL1&~(0b11<<6))|(0b01<<6)); //Configura o Clock: PCLKI_I2C1 = 100MHz

	// Configura os pinos P0.0 e P0.1 para SDA1 e SCL1 respectivamente
	LPC_PINCON->PINSEL0 |= (0b11<<0|0b11<<2);

	//Desativando os resistores de pull-up e pull-down pinos 0.0 e 0.1
	LPC_PINCON->PINMODE0 = ((LPC_PINCON->PINMODE0&~(0b11<<0|0b11<<2))|(0b10<<0|0b10<<2));

	//Modo dreno aberto em P0.0 e P0.1
	LPC_PINCON->PINMODE_OD0 |= (1<<0|1<<1);

	//PCLK_I2C1 = 100 MHz
	//Limpa o registro de controle da interface I2C1
	//Frequencia e sinal de clock SCL = 400KHz
	LPC_I2C1->I2CONCLR = I2EN|STA|SI|AA;
	LPC_I2C1->I2SCLH = 125;
	LPC_I2C1->I2SCLL = 125;

	//Setando o modo master
	LPC_I2C1->I2CONSET = I2EN;

}

void COND_STOP()
{

	LPC_I2C1->I2CONSET = STO; //Solicita uma condição de STOP
	LPC_I2C1->I2CONCLR = SI; //Limpa o flag bit de interrupção SI
	while(LPC_I2C1->I2CONSET&STO); //Espera a geração de um STOP

}

unsigned char I2C_ENVIA_DADO(unsigned char dado_i2c)
{

	LPC_I2C1->I2DAT = dado_i2c; //Carrega os dados no registro de deslocamento.
	LPC_I2C1->I2CONCLR = SI; //Limpa o flag bit de interrupção SI.

	while(!(LPC_I2C1->I2CONSET&SI)); //Espera a transmissão dos dados e o recebimendo o ack

	if(LPC_I2C1->I2STAT == 0x28)
	{//Verifica estado da interface
		return 0; //A Interface enviou o dado e recebeu um ACK
	}
	else
	{
		return 1; //A Interface não conseguiu enviar o dado e recebeu um NACK
	}

}

unsigned char I2C_RECEBE_DADO(unsigned char *dado_i2c, unsigned char acknowledge)
{

	if(acknowledge) //Verifica qual sinal de acknowledge a interface deve retornar.
	{
		LPC_I2C1->I2CONSET = 1<<2; //Seta o bit AA. A interface retornará um ACK após receber o dado.
	}

	else
	{
		LPC_I2C1->I2CONCLR = 1<<2; //Limpa o bit AA. A interface retornará um NACK após receber o dado.
	}

	LPC_I2C1->I2CONCLR = SI; //Limpa o flag bit de interrupção SI.


	while(!(LPC_I2C1->I2CONSET&SI)); //Aguarda a recepção de um dado e a transmissão do acknowledge

	*dado_i2c = LPC_I2C1->I2DAT; //Armazena o dado recebido pela interface.

	if(LPC_I2C1->I2STAT == 0x50) //Verifica o estado da interface.
		return 0; //Retornou um ACK
	else
		return 1; //Retornou um NACK

}

unsigned char I2C_ADD_MEMORIA(unsigned char add, unsigned char operacao) // Funcao que leva o endereço do slave e a operação
{

	if(LPC_I2C1->I2CONSET&SI) //Verifica se o flag bit de interrupcao está setado.
	{
		LPC_I2C1->I2CONSET = STA; //Solicita a geração de um START
		LPC_I2C1->I2CONCLR = SI; //Limpa o flag bit de interrupção.
	}

	else
		LPC_I2C1->I2CONSET = STA; //Solicita a geração de um START

	while((LPC_I2C1->I2STAT!=0x0008)&(LPC_I2C1->I2STAT!=0x0010)); //Esperando a geração de um START
	LPC_I2C1->I2DAT = ((add<<1)|(operacao&1)); //Carrega o registro de deslocamento com o endereço do slave e a operação.

	LPC_I2C1->I2CONCLR = SI|STA; //Limpa o flag bit de interrupção e o START

	while(!(LPC_I2C1->I2CONSET&SI)); //Esperando a transmissão do dado e o recebimento do ACK

	switch(LPC_I2C1->I2STAT)
	{
		case 0x18:
			//printf("Recebeu o ACK");
			return 0; //Recebeu um ACK depois de uma escrita

		case 0x20:
			//printf("recebeu o NACK");
			return 1; //Recebeu um NACK depois de uma escrita

		case 0x40:
			//printf("Recebeu ACK");
			return 0; //Recebeu um ACK depois de uma leitura

		case 0x48:
			//printf("Recebeu NACK");
			return 1; //Recebeu um NACK depois de uma leitura

		default:
			//printf("%d", (LPC_I2C1->I2STAT));
			COND_STOP();
			return 2; //Gera uma condição de STOP

	}

}

unsigned char I2C_ESCREVE(unsigned char add, unsigned char dado) //O dado que se quer enviar corresponde a ação que se deseja executar
{
	if(!I2C_ADD_MEMORIA(add, OP_ESCRITA)) //Envia o endereço do slave e o tipo de operação
	{
		if(!I2C_ENVIA_DADO(dado)) //Envia o dado que se quer escrever
		{
			COND_STOP(); //Gera a condição de STOP
			return 0; //Sensor retornou um ACK
		}

		else
		{
			COND_STOP(); //Gera uma condição de STOP
			return 1; //Sensor retornou um NACK
		}
	}
	COND_STOP(); //Gera uma condição de STOP
	return 2; //Ocorreu algum erro e a operação foi interrompida.

}

unsigned char I2C_LE(unsigned char add, unsigned char *dado_rec, unsigned char *dado_rec2)
{

	if(!I2C_ADD_MEMORIA(add, OP_LEITURA)) //Gera condição de START e envia o endereço do slave e o tipo de operação
	{
		//printf("Entrou no if");
		I2C_RECEBE_DADO(dado_rec, ACK); //A interface recebe a primeira parte do dado e retorna um ACK
		I2C_RECEBE_DADO(dado_rec2, NACK); //A interface recebe a segunda parte dos dados e retorna um NACK
		COND_STOP(); //Gera uma condição de STOP
		//dado1 = dado_rec;
		//dado2= dado_rec2;
		return 0; //Operação finalizada com sucesso.
	}
	COND_STOP(); //Gera uma condição de STOP
	return 2; //Ocorreu algum erro e a operação foi interrompida.

}
