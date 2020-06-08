//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

//
// The central computer in the ALOHAnet network.
//
simple Server
{
    parameters:
        @display("i=device/antennatower_l");
        @signal[receiveBegin](type="long");  // increases with each new frame arriving to the server and drops to 0 if the channel becomes finally idle
        @signal[receive](type="long");  // for successful receptions (non-collisions): 1 at the start of the reception, 0 at the end of the reception
        @signal[collision](type="long"); // the number of collided frames at the beginning of the collision period
        @signal[collisionLength](type="simtime_t");  // the length of the last collision period at the end of the collision period
        @signal[channelState](type="long");

        @statistic[serverChannelState](source="channelState";title="Channel state";enum="IDLE=0,TRANSMISSION=1,COLLISION=2";record=vector);
        @statistic[receiveBegin](source="receiveBegin"; record=vector?; interpolationmode=sample-hold; title="receive begin");
        @statistic[channelUtilization](source="timeavg(receive)"; record=last; interpolationmode=linear; title="channel utilization");
        @statistic[collisionMultiplicity](source=collision; record=vector?,histogram; title="collision multiplicity");
        @statistic[collisionLength](record=vector?,histogram,mean,sum,max; title="collision length");
        @statistic[receivedFrames](source="sum(receive)"; record=last; title="received frames");
        @statistic[collidedFrames](source="sum(collision)"; record=last; title="collided frames");
        
        string vectorOutput;
        
        int minHosts;
        int maxHosts;
        int HostStepSize;
        double RX_TX_switching_time @unit(us);
        double HostMinimumSensingTime @unit(us);
        double carrier_sense_duration @unit(us);
        bool cs_enabled;
        
        int mode;
        int test_mode;
        int haltOnPacketNumberSent;
        double txRate @unit(bps);         // transmission rate
        double radioDelay @unit(s);       // propagation delay of radio link
        volatile int pkLenBits @unit(b);  // packet length in bits
        volatile int ACKLenBits @unit(b);  // packet length in bits
        double delay_after_ack @unit(us);
        int numberHostsPlotModes;
        int frame_overhead;	//overhead of a data frame (used in test_mode == 2)
        
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
        bool ClockDriftPlotMode;
        int ClockDriftPlotStepNumber;
        
        bool ExternalInterferenceEnable;
        double ExternalInterferenceDutyCycle;
        bool ExternalInterferencePlotMode;
        int ExternalInterferencePlotStepNumber;
		
    gates:
        input in @directIn;
}

