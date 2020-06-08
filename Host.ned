//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//


//
// A computer in the ALOHAnet network.
//
simple Host
{
    parameters:
        @signal[state](type="long");
        @statistic[radioState](source="state";title="Radio state";enum="IDLE=0,TRANSMIT=1";record=vector);
        double txRate @unit(bps);         // transmission rate
        double radioDelay @unit(s);       // propagation delay of radio link
        volatile int pkLenBits @unit(b);  // packet length in bits
        int mode;
        int test_mode;
        
        int minHosts;
        int maxHosts;
        int HostStepSize;
        double RX_TX_switching_time @unit(us);
        double HostMinimumSensingTime @unit(us);
        double carrier_sense_duration @unit(us);
        bool cs_enabled;
        volatile int ACKLenBits @unit(b);  // packet length in bits
        double delay_after_ack @unit(us);
        
        //Strawman configuration
        int Strawman_max_repetition;
        double Strawman_max_contention_length @unit(ms);
        double Strawman_min_contention_length @unit(ms);
        int Strawman_max_contention_slots;
        int Strawman_CR_length @unit(b);				
        int Strawman_decision_length @unit(b);
        double Strawman_contention_start_delay @unit(ms);

        //Bin-MAC configuration
        int BTCR_query_length @unit(b);
        double BTCR_delay_after_query @unit(ms);
        double BTCR_wait_duration @unit(ms);
        
        //proposed configuration
		int Proposed_ACK_length @unit(b);										
		int Proposed_slot_number;
		int Proposed_recovery_mode;
		
		//CSMA configuration
		double CSMA_backoff_time @unit(us);
		int CSMA_max_retransmissions;
		int CSMA_max_backoffs;
		int CSMA_wc_min;
		int CSMA_wc_max;
		int CSMA_min_BE;
		int CSMA_max_BE;
		
		bool ClockDriftEnabled;
        double ClockDriftRangePercent;
        bool ClockDriftPlotMode;
        int ClockDriftPlotStepNumber;
        
        bool ExternalInterferenceEnable;
        double ExternalInterferenceDutyCycle;
        bool ExternalInterferencePlotMode;
        int ExternalInterferencePlotStepNumber;
		
        @display("i=device/pc_s");

    gates:        
        input in @directIn;
}

