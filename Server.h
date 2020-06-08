//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//


#ifndef __ALOHA_SERVER_H_
#define __ALOHA_SERVER_H_

#define MAX_NUMBER_OF_NODES     512

#include <omnetpp.h>
#include <stdlib.h>
#include <inttypes.h>
#include <time.h>

struct packetCount                    //for internal use (not statistics) temporary (counts packets within a sequence)
{
    int packetsReceivedTotal;         //number of received packets (does also count defective packets)
    int packetsReceivedGood;          //number of packets that arrived without collision
    int firstPacketIndex;             //Index of the first packet of a message that has been received
    int packetsSkipped;               //number of packets that have been skipped in mode4
};

struct result_t                       //for statistics: holds the results for each iteration
{
    long int numberOfNodes;
    long int numberOfPackets;
    int maxHosts;
    int packet_size;
    long int packetSmallReceived;
    unsigned long packetSmallLost;
    unsigned long int sequencesReceived;
    unsigned long int sequencesLost;
    unsigned long int sequencesTotal;   //number of sequences sent
    double deadline_missed_counter;
    long int ACKsLost;
    long int ACKsTransmitted;               //total number of ACKs that were transmitted
    long int packetsSkipped;
    int min;
    int max;
    float mean;
    float stddev;

    double delay_avg;
    double delay_min;
    double delay_max;
    double rx_time_avg;                    //time that nodes were in receiver mode
    double rx_time_min;
    double rx_time_max;
    double tx_time_avg;
    double tx_time_min;
    double tx_time_max;
};





namespace aloha {

/**
 * Aloha server; see NED file for more info.
 */
class Server : public cSimpleModule
{
  private:
    // state variables, event pointers
    //bool channelBusy;                                       // true: channel is busy
    bool receivingPaket;                                    // true: packet is being received at the moment (and causes channel to be busy)
    simtime_t radioDelay;
    double txRate;
    cPar *pkLenBits;
    std::string vectorOutput;

    enum ack_t {ACK=0, Strawman_CR, Strawman_Not, NAK, BTCR_ACK, BTCR_NAK, PROPOSED_ACK,
        PROPOSED_NAK, PROPOSED_NEXT_SLOT, CSMA_ACK, CSMA_NAK, FORCE_SLEEP} ACK_type; //type of ACK: CR/Notification etc
    int host_id_list[MAX_NUMBER_OF_NODES];                  //list of random IDs of current hosts

    cMessage *endRxEvent;
    cMessage *restartEvent;
    cMessage *ACKStartEvent;
    cMessage *ACKFinishedEvent;
    cMessage *simtime_event;

    cPar *ACKLenBits;
    double ACK_duration;
    bool send_n_packets;                                    //true: hosts send n packets
    double carrier_sense_duration;
    bool cs_enabled;
    double data_duration;
    double delay_after_ack;

    double restart_interval;
    long restart_counter;                                   //counts the number of restart events
    int repetition, id_temp, ACK_host_id;
    bool senderACKRequest;
    bool ACK_currently_send;
    bool ACK_NAK_corrupt;               //true if interference occurred during sending
    bool sleep_mode_enabled;                    //true if server finished cycle and now sleeps (does not accept any packets during sleeping)
//    double data_duration_minimum;       //l_min: length of empty slot (proposed and BTCR)
//    double data_duration_maximum;       //l_max: maximum length of slot (for shifting NAK after interference pulse) (proposed and BTCR)

    simtime_t recvStartTime;
    enum {IDLE=0, TRANSMISSION=1, COLLISION=2};
    simsignal_t channelStateSignal;

    packetCount packetList[MAX_NUMBER_OF_NODES];            //array mit Infos über empfangene und verlorene Pakete jedes Knotens
    int nodesFinishedTransmission[MAX_NUMBER_OF_NODES];     //enthält ID der Knoten, die seit letztem rxEndEvent fertig geworden sind
    bool receiveCounter[MAX_NUMBER_OF_NODES];               //true if a node (index is node id) finished transmission in probe cycle. Used to count successrate (sequencesReceived) and prevent multiple counts, for example, if ACK has been corrupted and node tries to send again later
    int nodesFinishedTransmissionNumber;                    //enthält Anzahl der Knoten, die seit letztem rxEndEvent fertig geworden sind
    int minHosts;
    int maxHosts;
    int maxHosts_old;                                       //stores initial value of maxHosts, used for special modes, such as maxHost varying for constant current_active_nodes (useful for proposed + BTCR)
    int HostStepSize;
    int numberHostsPlotModes;

    int mode;
    int test_mode;

    int haltOnPacketNumberSent;
    double max_simtime_per_iteration;
    bool interference_in_empty_slot_flag;               //a single interference pulse was detected as a collided packet (occurred within first t_CCA time of slot) (proposed and BTCR only)
    double slot_start_timestamp;
    double interference_start_timestamp;                //timestamp of start of interference pulse
    double interference_stop_timestamp;                 //and when it stops

    int currentActiveNodes;                             //the number of nodes which are currently active (in this iteration)
    int currentPaketCountPerSequence;                   //the number of packets within a sequence (k <= n) (in this iteration)

    //test mode 2: different packet sizes
    int test_packet_size_start;
    int test_packet_size_end;
    int test_packet_size_current;
    int test_packet_size_step;
    int frame_overhead;                             //packet header overhead


    //TODO remove some redundant variables?
    long int packetSmallReceived;               //number of packets (each sequence contains a number of packets)
    long int packetSmallLost;
    long int sequencesReceived;                 //number of received sequences (also contains non successful sequences)
    long int sequencesLost;                     //number of non successful sequences
    long int sequencesTotal;                    //number of total sequences sent (used for calculating loss)
    long int currentCollisionNumFrames;
    long int ACKsLost;
    long int packetsSkipped;
    long int packetsSkippedTotal;
    int id_list_packets_collided[10001];        //contains the host-IDs, whose packets collided simultaneously
    int index;
    long int delay_counts;
    long int total_packets_transmitted;         //used for calculating the energy
    long int total_packets_skipped;
    long int total_acks_transmitted;


    bool lastPacketGood;        //flag: true: last packet (lastID) was received successfully
                                //      false: last packet collided. Used for collision counting
    unsigned int lastID;        //contains the ID of the last sender.Used for collision counting
    char *fullPackets;          //contains the ID of all Nodes that finished the package count. Is cleared after rxEndEvent

    // statistics
    simsignal_t receiveBeginSignal;
    simsignal_t receiveSignal;
    simsignal_t collisionLengthSignal;
    simsignal_t collisionSignal;

    double RX_TX_switching_time;
    double HostMinimumSensingTime;
    double HostSensingTime;                 //Time that host sense the channel for activity


    //Strawman configuration
    int Strawman_max_repetition;
    double Strawman_max_contention_length;
    double Strawman_min_contention_length;
    int Strawman_min_contention_slot_number;
    int Strawman_max_contention_slots;
    double Strawman_contention_slot_length;
    int Strawman_CR_length;
    int Strawman_decision_length;           //number of bytes of the decision (notification) packet
    double Strawman_decision_duration;      //in seconds
    double Strawman_CR_duration;
    bool Strawman_waiting_for_contention;   //true: waiting for contention messages from nodes; false: normal operation (data receiving)
    double Strawman_longest_cp;             //holds the length of the longest cp pulse from the contention resolution (will be sent in the decision packet)
    cMessage *Strawman_cp_end;              //indicates the end of contention pulse cycle (finished receiving all cp)
    cMessage *Strawman_not_timeout;         //timeout event for notification messages (when there is no data coming after a notification)
    int Strawman_not_timeout_counter;
    double Strawman_contention_start_delay; //delay after notification before contention message starts being sent (should be longer than RX_TX)
    double Strawman_cp_start_timestamp;     //time when contention window starts, needed for processing external interference
    double Strawman_cp_stop_timestamp;      //time when contention windows ends (longest contention packet), used for processing external interference
    int Strawman_CR_repetition_counter;     //counts CR messages followed by timeouts (used to abort cycle)

    //Bin-MAC configuration
    int BTCR_query_length;
    double BTCR_query_duration;
    double BTCR_delay_after_query;
    int BTCR_higher_id, BTCR_lower_id;      //current ID range
    int BTCR_last_successful_id;            //ID of last successful transmission
    double BTCR_wait_duration;
    int BTCR_split_selected;                //can be 1 or 2: 1:= first split selected (higher ID range of split); 2:= second split selected (lower ID range); >2 means both slots were empty -> sleep
    cMessage *BTCR_query_timeout;
    int BTCR_split_count;

    //proposed configuration
    int Proposed_ACK_length;
    double Proposed_ACK_duration;
    double Proposed_slot_duration;
    int Proposed_slot_number;
    int Proposed_slot_number_start;                 //for test_mode == 3: increase slot number from start to stop
    int Proposed_slot_number_stop;
    int Proposed_current_slot;                      //current slot number (starts from 1), used for counting
    int Proposed_higher_id;
    int Proposed_lower_id;
    bool Proposed_TDMA_active;
    cMessage *Proposed_slot_timeout;                //timeout to check if slot is empty
    int Proposed_slot_number_current;               //current slot number of TDMA cycle (can be reduced if id range is too small for a bigger split)
    bool Proposed_slot_contains_single_ids;         //sub-slots contain 1 ID each. If further interference happens, these are repeated directly (up to two times)
    int Proposed_single_slot_repetition_counter;    //counts number of times that a single-ID slot has been repeated
    bool Proposed_first_cycle;                      //true: cycle is started the first time
    double Proposed_timeout_duration;
    double Proposed_max_slot_duration;
    int Proposed_recovery_mode;
    int Proposed_recovery_cycle_count;              //hold the number of recovery cycles that have been processed so far in this probe-cycle (not the allowable max)
    int Proposed_recovery_cycles_max;
    bool Proposed_send_recovery_ids;                //
    bool Proposed_recovery_slot;                    //true: current slot is recovery (gets reset in next ACK/NAK
    bool Proposed_last_slot_was_recovery;           //true: last slot was recovery slot. Used to avoid skipping the slot after the recovery slot
    bool Proposed_BTCR_mode;                             //Proposed mode is in BTCR mode

    struct proposed_id_t {
        int higher;
        int lower;
    };

    proposed_id_t Proposed_slot_list[1024];
    void Proposed_generate_slots();
    void Proposed_handle_collision();           //collision handler
    void Proposed_finish_cycle();               //all IDs have been processed -> call this function to prepare for restart
    void Strawman_finish_cycle();               //sink goes to sleep
    void BTCR_finish_cycle();


    //CSMA variables ///////////////////////////////////////////////////////
    double CSMA_backoff_time;
    int CSMA_max_retransmissions;
    int CSMA_max_backoffs;
    int CSMA_wc_min;
    int CSMA_wc_max;


    //clock drift
    bool ClockDriftEnabled;
    double ClockDriftRangePercent;
    bool ClockDriftPlotMode;
    int ClockDriftPlotStepNumber;
    int ClockDriftPlotCurrentStepNumber;           // counter for current iteration, used to calculate the size of clockdrift
    double ClockDriftPlotDriftRange_temp;
    double Clock_drift_current_value;             //current clock drift that has been (randomnly) selected between [0,ClockDriftRangePercent]
    double periodTime_original;                     //period time without any drift etc. The clock drift modifies periodTime
    bool deadline_missed;
    double deadline_missed_counter;

    //external interference
    bool ExternalInterferenceEnable;
    double ExternalInterferenceDutyCycle;
    double ExternalInterferenceCurrentDutyCycle;
    bool ExternalInterferencePlotMode;
    int ExternalInterferencePlotStepNumber;
    int ExternalInterferencePlotCurrentStepNumber; //iteration counter
    cMessage *ExternalInterferenceEvent;
    double ExternalInterference_active_time;    //used for getting the right duty cycle
    double ExternalInterference_idle_time;
    double ExternalInterference_active_min;     //min max times for shuffling active time
    double ExternalInterference_active_max;
    bool ExternalInterference_is_active;   //true: channel is distorted


    //statistics
    long packets_received_total;                //for one iteration cycle only


    //recording
    cLongHistogram *hopCountStats;
    result_t statsAll[200];            //hopCountStats of all iterations
    int storage_index;
    int storage_iteration_number;
    cDoubleHistogram *rx_time;           //receive times (receiver active of hosts)
    cDoubleHistogram *tx_time;           //transmit times of hosts
    cDoubleHistogram *delay;             //message delay until successful reception
    double delay_temp;
    double tx_time_temp;
    double rx_time_temp;


    void initVariables();
    void sendACKPacket(char *name, int host_id, int type);

  public:
    Server();
    virtual ~Server();

    void start_next_iteration();    //start next step of the simulation, for example, increase host numbers, packet sizes, etc.
    bool channel_busy2();           //simple carrier sensing

  protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
    virtual void finish();
};

}; //namespace

#endif

