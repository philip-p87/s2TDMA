//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#ifndef __ALOHA_HOST_H_
#define __ALOHA_HOST_H_



#include <omnetpp.h>
#include <time.h>

namespace aloha {



/**
 * Aloha host; see NED file for more info.
 */
class Host : public cSimpleModule
{
  private:
    simtime_t radioDelay;
    simtime_t delay_time_stamp;     //holds the time point when the node starts sending its first packet. Used to calculate message_delay
    double message_delay;           //delay from triggering the node until the end of the first successfully received packet
    double tx_time;
    double rx_time;                 //time a node spent in rx mode. Used to calculate energy consumption
    simtime_t delay_time_buf;
    double txRate;
    cPar *iaTime;
    cPar *pkLenBits;
    double data_duration;
    cPar *ACKLenBits;
    double timeout_duration;
    double delay_after_ack;
    int mode;


    int minHosts;
    int maxHosts;
    int HostStepSize;
    int current_number_of_active_hosts;
    int packetNumberMax;                //max packets in a sequence


    int packetNumber;           //counting variable: counts packets in a sequence
    int packet_retry_number;    //number of times a packet has collided in the past and now tries to retransmit
    char packet_name[64];
    bool ACK_received;

    enum ack_t {ACK=0, Strawman_CR, Strawman_Not, NAK, BTCR_ACK, BTCR_NAK, PROPOSED_ACK,
        PROPOSED_NAK, PROPOSED_NEXT_SLOT, CSMA_ACK, CSMA_NAK} ACK_type; //type of ACK: CR/Notification etc

    long int transmissions_successful;        //total number of successfully received packets
    long int transmissions_failed;            //number of failed packets (collected from host)
    bool ACK_NAK_empty;     //true if ACK was sent in empty slot
    bool ACK_NAK_corrupt;   //true if interference occurred during sending
    bool desync;
    bool host_id_carrier_sense[512];


    //Strawman configuration
    int Strawman_max_repetition;
    double Strawman_max_contention_length;
    double Strawman_min_contention_length;
    int Strawman_min_contention_slot_number;
    int Strawman_max_contention_slots;
    double Strawman_contention_slot_length;
    int Strawman_CR_length;
    int Strawman_decision_length;
    double Strawman_CR_duration;            //in seconds
    double Strawman_decision_duration;
    double Strawman_cp_length;              //length of own cp pulse
    double Strawman_cp_length_decision;     //received length from notification message
    bool Strawman_cp_active;                //bool send cp message
    double Strawman_contention_start_delay; //delay after notification before contention message starts being sent (should be longer than RX_TX)

    //Bin-MAC configuration
    int BTCR_query_length;
    double BTCR_delay_after_query;
    double BTCR_wait_duration;

    //proposed configuration
    int Proposed_ACK_length;
    double Proposed_ACK_duration;
    int Proposed_slot_number;

    int Proposed_host_slot;             //slot number of node
    int Proposed_current_slot;          //current slot that is now in use. When slot_number = current_slot we transmit (starts from 1)
    bool Proposed_TDMA_active;          //flag, true: TDMA collision resolution is active (is used to distinguish between ACK and when to increase current slot number)
    int Proposed_higher_id;
    int Proposed_lower_id;
    int Proposed_slot_number_current;
    bool Proposed_subcycle_finished;    //used to inform host about sub cycle finish (to recalculate slot range, number, etc.)
    int Proposed_recovery_mode;
    bool Proposed_BTCR_mode;                             //Proposed mode is in BTCR mode

    struct proposed_id_t {
        int higher;
        int lower;
    };

    proposed_id_t Proposed_slot_list[1024];
    void Proposed_fill_slot_list();
    void Proposed_calculate_host_slot();


    //CSMA variables ///////////////////////////////////////////////////////
    double CSMA_backoff_time;
    int CSMA_max_retransmissions;
    int CSMA_max_backoffs;
    int CSMA_wc_min;
    int CSMA_wc_max;
    int CSMA_min_BE;
    int CSMA_max_BE;

    int CSMA_retry_number;                  //counts retry number, used to compare against max retry number
    int CSMA_backoff_number;                //counts the number of back-off attempts
    bool CSMA_cs_done;                      //carrier sensing has been completed


    //uses carriersenseEndEvent
    double CSMA_get_random_backoff();
    void CSMA_retry(double delay_);    //will resent the packet after random backoff; extra_delay can be used to ad RX_TX_switching times


    //carrier sense variables ///////////////////////////////////////////////////////
    double ACK_duration;
    double RX_TX_switching_time;
    double HostMinimumSensingTime;
    double carrier_sense_duration;
    bool cs_enabled;

    double HostSensingTime;                 //Time that host sense the channel for activity
    bool carrier_sense_done;                //mode4: finished sensing the channel
    bool channel_was_busy_during_sensing;   //mode4: channel was busy during sensing
    bool cs_active;                         //true: node is currently doing a carrier sense
    bool channel_is_busy;                   //set by server: true: channel is used, false: channel is free


    bool ClockDriftEnabled;
    double ClockDriftRangePercent;
    bool ClockDriftPlotMode;
    int ClockDriftPlotStepNumber;
    int ClockDriftPlotCurrentStepNumber;           // counter for current iteration, used to calculate the size of clockdrift
    double ClockDriftPlotDriftRange_temp;
    double Clock_drift_current_value;                       //current clock drift that has been (randomnly) selected between [0,ClockDriftRangePercent]
    double periodTime_original;                             //period time without any drift etc. The clock drift modifies periodTime
    double Clock_drift_calculate_period(double period);     //takes period and calculates drift
    void Clock_drift_generate_new_random_drift();           //shuffles new random values for clock drift
    //double periodTimeDrifted;

    bool ExternalInterferenceEnable;
    double ExternalInterferenceDutyCycle;
    bool ExternalInterferencePlotMode;
    int ExternalInterferencePlotStepNumber;

    // state variables, event pointers etc
    cModule *server;
    cMessage *endRxEvent;
    cMessage *endTxEvent;
    cMessage *timeoutEvent;
    cMessage *endCarrierSenseEvent;
    enum {SLEEP=0, IDLE, TRANSMIT, TRANSMIT_FINISHED, WAITED_PERIOD, CONTENTION, WAITING_FOR_DATA, WAIT_FOR_NEXT_CONTENTION, WAIT_FOR_REPLY, WAIT_TO_GET_SELECTED, START_CS} state;
    simsignal_t stateSignal;


    void calculatePeriods(int maxPaketNumber, int numberOfTransmittingNodes);
    void sendPacket(char *name, double delay_, double duration_);
    void StrawmanSendContentionPacket(double length);
    double max(double a, double b);
    void reset_variables();

  public:
    Host();
    virtual ~Host();

    void wakeup(double data_duration_);      //will wake up the node which starts transmitting
    void sleep();       //node goes to sleep mode (usually called from within the node, not from server)

    void activate();                    //will activate the node: starts sending
    void stop_transmission();
    void change_transmission_scheme(int numberOfTransmittingNodes);  //changes the transmission scheme to next iteration. Called by Server node
    void next_plot_step(int iteration_number);

    void self_message_proposed(cMessage *msg);
    void self_message_strawman(cMessage *msg);
    void self_message_btcr(cMessage *msg);
    void self_message_CSMA(cMessage *msg);
    void ack_was_lost(bool only_ack_was_lost);  //only_ack_was_lost = true: message has been received successfully, but ACK got lost
                                                //false: message was corrupt

    void update_channel_state(bool channel_busy);  //used by server to inform host, whether channel is currently busy or not
    bool carrier_sense();                          //performs carrier sense

    long int collect_and_clear_transmission_successful(); //For Server: get number of successfully delivered packets
    long int collect_and_clear_transmission_failed();

    void TDMA_synchronize();


  protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
};

}; //namespace

#endif

