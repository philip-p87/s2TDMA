//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//


//
// The Aloha network consists of hosts which talk to the central "server" via
// the Aloha or Slotted Aloha protocol
//
network Aloha
{
    parameters:
        int minHosts;
        int maxHosts;  // number of hosts
        int HostStepSize;
        int mode;		//mode of operation: 1 normal single packet; 2=random multiple packets; 3=our algorithm
        int test_mode;
        double txRate @unit(bps);  // transmission rate
        double radioDelay @unit(s);       // propagation delay of radio link
        volatile int pkLenBits @unit(b);  // packet length in bits
        volatile int ACKLenBits @unit(b);  // packet length in bits
        double RX_TX_switching_time @unit(us);
        double HostMinimumSensingTime @unit(us);
        double carrier_sense_duration @unit(us);
        bool cs_enabled;
        int haltOnPacketNumberSent;
        double delay_after_ack @unit(us);
        int numberHostsPlotModes;

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
		int Proposed_slot_number_start;
		int Proposed_slot_number_stop;
		int Proposed_recovery_mode;
		int Proposed_recovery_cycles_max;
		
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
        int ClockDriftPlotStepNumber;
        bool ClockDriftPlotMode;
        
        bool ExternalInterferenceEnable;
        double ExternalInterferenceDutyCycle;
        int ExternalInterferencePlotStepNumber;
        bool ExternalInterferencePlotMode;
        
        
        
		@display("bgi=background/terrain,s;bgb=718,342");


    submodules:
        server: Server {
            mode = mode;
            test_mode = test_mode;
            minHosts = minHosts;
            maxHosts = maxHosts;
            HostStepSize = HostStepSize;
            haltOnPacketNumberSent = haltOnPacketNumberSent;
            txRate = txRate;
            radioDelay = radioDelay;
            pkLenBits = pkLenBits;
            RX_TX_switching_time = RX_TX_switching_time;
            HostMinimumSensingTime = HostMinimumSensingTime;
            carrier_sense_duration = carrier_sense_duration;
            cs_enabled = cs_enabled;
            ACKLenBits = ACKLenBits;
            delay_after_ack = delay_after_ack;
            numberHostsPlotModes = numberHostsPlotModes;
            
            //Strawman configuration
            Strawman_max_repetition = Strawman_max_repetition;
            Strawman_max_contention_length = Strawman_max_contention_length;
            Strawman_min_contention_length = Strawman_min_contention_length;
            Strawman_max_contention_slots = Strawman_max_contention_slots;
            Strawman_CR_length = Strawman_CR_length;				
            Strawman_decision_length = Strawman_decision_length;
            Strawman_contention_start_delay = Strawman_contention_start_delay;
    
            //Bin-MAC configuration
            BTCR_query_length = BTCR_query_length;
            BTCR_delay_after_query = BTCR_delay_after_query;
            BTCR_wait_duration = BTCR_wait_duration;
            
            //proposed configuration
    		Proposed_ACK_length = Proposed_ACK_length;										
    		Proposed_slot_number = Proposed_slot_number;
    		Proposed_slot_number_start = Proposed_slot_number_start;
    		Proposed_slot_number_stop = Proposed_slot_number_stop;
    		Proposed_recovery_mode = Proposed_recovery_mode;
    		Proposed_recovery_cycles_max = Proposed_recovery_cycles_max;
    		
    		//CSMA configuration
    		CSMA_backoff_time = CSMA_backoff_time;
		    CSMA_max_retransmissions = CSMA_max_retransmissions;
		    CSMA_max_backoffs = CSMA_max_backoffs;
		    CSMA_wc_min = CSMA_wc_min;
		    CSMA_wc_max = CSMA_wc_max;
		    CSMA_min_BE = CSMA_min_BE;
		    CSMA_max_BE = CSMA_max_BE;
            
            
            ClockDriftEnabled = ClockDriftEnabled;
        	ClockDriftRangePercent = ClockDriftRangePercent;
        	ClockDriftPlotMode = ClockDriftPlotMode;
        	ClockDriftPlotStepNumber = ClockDriftPlotStepNumber;
        	
        	ExternalInterferenceEnable = ExternalInterferenceEnable;
        	ExternalInterferenceDutyCycle = ExternalInterferenceDutyCycle;
        	ExternalInterferencePlotMode = ExternalInterferencePlotMode;
        	ExternalInterferencePlotStepNumber = ExternalInterferencePlotStepNumber;

            @display("p=357,65");
        }
        host[maxHosts]: Host {
            mode = mode;
            test_mode = test_mode;
            minHosts = minHosts;
            maxHosts = maxHosts;
            HostStepSize = HostStepSize;
            txRate = txRate;
            radioDelay = radioDelay;
            pkLenBits = pkLenBits;
            RX_TX_switching_time = RX_TX_switching_time;
            HostMinimumSensingTime = HostMinimumSensingTime;
            carrier_sense_duration = carrier_sense_duration;
            cs_enabled = cs_enabled;
            ACKLenBits = ACKLenBits;
            delay_after_ack = delay_after_ack;
            
            //Strawman configuration
            Strawman_max_repetition = Strawman_max_repetition;
            Strawman_max_contention_length = Strawman_max_contention_length;
            Strawman_min_contention_length = Strawman_min_contention_length;
            Strawman_max_contention_slots = Strawman_max_contention_slots;
            Strawman_CR_length = Strawman_CR_length;				
            Strawman_decision_length = Strawman_decision_length;
            Strawman_contention_start_delay = Strawman_contention_start_delay;
    
            //Bin-MAC configuration
            BTCR_query_length = BTCR_query_length;
            BTCR_delay_after_query = BTCR_delay_after_query;
            BTCR_wait_duration = BTCR_wait_duration;
            
            //proposed configuration
    		Proposed_ACK_length = Proposed_ACK_length;										
    		Proposed_slot_number = Proposed_slot_number;
    		Proposed_recovery_mode = Proposed_recovery_mode;
    		
    		//CSMA configuration
    		CSMA_backoff_time = CSMA_backoff_time;
		    CSMA_max_retransmissions = CSMA_max_retransmissions;
		    CSMA_max_backoffs = CSMA_max_backoffs;
		    CSMA_wc_min = CSMA_wc_min;
		    CSMA_wc_max = CSMA_wc_max;
		    CSMA_min_BE = CSMA_min_BE;
		    CSMA_max_BE = CSMA_max_BE;
            
            ClockDriftEnabled = ClockDriftEnabled;
        	ClockDriftRangePercent = ClockDriftRangePercent;
        	ClockDriftPlotMode = ClockDriftPlotMode;
        	ClockDriftPlotStepNumber = ClockDriftPlotStepNumber;
        	
        	ExternalInterferenceEnable = ExternalInterferenceEnable;
        	ExternalInterferenceDutyCycle = ExternalInterferenceDutyCycle;
        	ExternalInterferencePlotMode = ExternalInterferencePlotMode;
        	ExternalInterferencePlotStepNumber = ExternalInterferencePlotStepNumber;
        	
        }
}

