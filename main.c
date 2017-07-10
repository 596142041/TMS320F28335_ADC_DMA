/*
 * main.c
 */
#include "DSP2833x_Device.h"     // Headerfile Include File
#include "DSP2833x_Examples.h"

#pragma DATA_SECTION(DMABuf1,"DMARAML4");
//volatile Uint16 DMABuf1[60];
volatile Uint16 DMABuf1[20][3];

volatile Uint16 *DMADest;
volatile Uint16 *DMASource;


Uint16 jieguo[3][10];
Uint16 i=0,j=0;

void pwmset();
void adcset();
void dmaset();
interrupt void dma();
interrupt void adcx();
interrupt void adcx()
{
	if(j>10)
	{
		EPwm1Regs.ETSEL.bit.SOCAEN=0;
		j=0;
	}
	DmaRegs.CH1.CONTROL.bit.PERINTFRC=1;
	jieguo[0][j]=((AdcRegs.ADCRESULT0)>>4);
	jieguo[1][j]=((AdcRegs.ADCRESULT1)>>4);
	j++;
	AdcRegs.ADCTRL2.bit.RST_SEQ1=1;
	PieCtrlRegs.PIEACK.bit.ACK1=1;
	AdcRegs.ADCST.bit.INT_SEQ1_CLR=1;
	EINT;
}




interrupt void dma()
{
	j++;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
	DMADest=&DMABuf1[0][0];
}
void pwmset()
{
	EPwm1Regs.TBPRD=50;//4us;
	EPwm1Regs.TBCTR=0;
	EPwm1Regs.TBPHS.all=0;

	EPwm1Regs.TBCTL.bit.FREE_SOFT=1;
	EPwm1Regs.TBCTL.bit.CLKDIV=1;//sys/12
	EPwm1Regs.TBCTL.bit.HSPCLKDIV=3;
	EPwm1Regs.TBCTL.bit.PRDLD=0;
	EPwm1Regs.TBCTL.bit.PHSEN=0;
	EPwm1Regs.TBCTL.bit.CTRMODE=0;
	EPwm1Regs.ETSEL.bit.SOCAEN=1;
	EPwm1Regs.ETSEL.bit.SOCASEL=2;
	EPwm1Regs.ETSEL.bit.SOCBEN=0;
	EPwm1Regs.ETPS.bit.SOCAPRD=1;


}

void adcset()
{
		InitAdc();
		AdcRegs.ADCTRL1.bit.ACQ_PS=1;
		AdcRegs.ADCTRL1.bit.CONT_RUN=0;
		AdcRegs.ADCTRL1.bit.CPS=1;
		AdcRegs.ADCTRL1.bit.SEQ_CASC=1;

		AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ=0;
		AdcRegs.ADCTRL2.bit.RST_SEQ1=1;
		AdcRegs.ADCTRL2.bit.SOC_SEQ1=0;
		AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1=1;
		AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1=0;
		AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1=1;
		AdcRegs.ADCTRL2.bit.EXT_SOC_SEQ1=0;
		AdcRegs.ADCTRL3.bit.SMODE_SEL=0;
		AdcRegs.ADCTRL3.bit.ADCCLKPS=3;//

		AdcRegs.ADCTRL3.bit.SMODE_SEL=0;
		AdcRegs.ADCTRL3.bit.ADCCLKPS=3;//12.5Mh;
		AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x00;//此通道采集三相电压中的A相电压  ADC_A0
		AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x01;//此通道采集三相电压中的B相电压  ADC_A1
		AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x08;//此通道采集三相电压中的C相电压  ADC_A2

		AdcRegs.ADCMAXCONV.bit.MAX_CONV1=2;
}

void dmaset()
{
		DMAInitialize();

		DMADest=&DMABuf1[0][0];
		DMASource= &AdcMirror.ADCRESULT0;
		DMACH1AddrConfig(DMADest,DMASource);
		DMACH1BurstConfig(2,1,3);
		DMACH1TransferConfig(2,0,0);
		DMACH1WrapConfig(0,0,0,1);//no use wrap;

		EALLOW;

			// Set up MODE Register:
			DmaRegs.CH1.MODE.bit.PERINTSEL =1;            // Passed DMA channel as peripheral interrupt source
			DmaRegs.CH1.MODE.bit.PERINTE = PERINT_ENABLE;               // Peripheral interrupt enable
			DmaRegs.CH1.MODE.bit.ONESHOT = ONESHOT_DISABLE;               // Oneshot enable
			DmaRegs.CH1.MODE.bit.CONTINUOUS =1;                    // Continous enable
			DmaRegs.CH1.MODE.bit.SYNCE = SYNC_DISABLE;                 // Peripheral sync enable/disable
			DmaRegs.CH1.MODE.bit.SYNCSEL = SYNC_SRC;               // Sync effects source or destination
			DmaRegs.CH1.MODE.bit.OVRINTE = OVRFLOW_DISABLE;         // Enable/disable the overflow interrupt
			DmaRegs.CH1.MODE.bit.DATASIZE =SIXTEEN_BIT;              // 16-bit/32-bit data size transfers
			DmaRegs.CH1.MODE.bit.CHINTMODE = CHINT_END ;                // Generate interrupt to CPU at beginning/end of transfer
			DmaRegs.CH1.MODE.bit.CHINTE =CHINT_ENABLE  ;                // Channel Interrupt to CPU enable

			// Clear any spurious flags:
			DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;                  // Clear any spurious interrupt flags
			DmaRegs.CH1.CONTROL.bit.SYNCCLR = 1;                    // Clear any spurious sync flags
			DmaRegs.CH1.CONTROL.bit.ERRCLR = 1;                      // Clear any spurious sync error flags

			// Initialize PIE vector for CPU interrupt:
			// Enable DMA CH1 interrupt in PIE
        EDIS;



}
unsigned long P_RAM_Init;  /*清ram用的指针*/
#define RAM_LEN        0x001000
#define RAM_Start_ADD     0xc000
void Ram_Init(void)
{

	for(P_RAM_Init=0 ; P_RAM_Init < RAM_LEN ; P_RAM_Init = P_RAM_Init+2 )
	{
		if ((unsigned long *)(RAM_Start_ADD + P_RAM_Init) !=  &P_RAM_Init)
		{
			*(unsigned long *)(RAM_Start_ADD + P_RAM_Init) = 0;
		}

	}
	P_RAM_Init=0;
}
void main()
{
        InitSysCtrl();

 	   EALLOW;
 		   GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
 		   GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 0;
 		   GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 0x00;
 		   GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;
 		   GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;
 		   GpioCtrlRegs.GPBDIR.bit.GPIO58 = 1;
 		   GpioCtrlRegs.GPBDIR.bit.GPIO59 = 1;
 		   GpioCtrlRegs.GPBDIR.bit.GPIO60 = 1;
 		   GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1;
 		   GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
 	   EDIS;
 	   GpioDataRegs.GPBCLEAR.all = 0xFFFFFFFF;
        DINT;
        InitPieCtrl();
        IER = 0x0000;
        IFR = 0x0000;
        InitPieVectTable();
        EALLOW;        // Allow access to EALLOW protected registers
             PieVectTable.DINTCH1= &dma;
        EDIS;
        // IER |= M_INT7 ;
        // PieCtrlRegs.PIEIER7.bit.INTx1=1;
         Ram_Init();
         for(i=0;i<4000;i++);
         pwmset();
         adcset();
         dmaset();
         EINT;

         EPwm1Regs.ETSEL.bit.SOCAEN=1;

         StartDMACH1();

         while(1);
}


