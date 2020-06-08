//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//


#include "Server.h"
#include "Host.h"

#include <iostream>
#include <fstream>


namespace aloha {

Define_Module(Server);
Host *host;

Server::Server()
{
    endRxEvent = NULL;
    hopCountStats = new cLongHistogram();
}

Server::~Server()
{
    cancelAndDelete(endRxEvent);
    cancelAndDelete(restartEvent);
    cancelAndDelete(ACKFinishedEvent);
    cancelAndDelete(ACKStartEvent);
    cancelAndDelete(ExternalInterferenceEvent);
    cancelAndDelete(Strawman_cp_end);
    cancelAndDelete(simtime_event);
    cancelAndDelete(BTCR_query_timeout);
    cancelAndDelete(Proposed_slot_timeout);
    cancelAndDelete(Strawman_not_timeout);
    free(packetList);

    delete delay;
    delete tx_time;
    delete rx_time;
}

void Server::initialize()
{
    gate("in")->setDeliverOnReceptionStart(true);

    vectorOutput = par("vectorOutput").str();
    EV << "vector output: " << vectorOutput.c_str() << endl;

    txRate = par("txRate");
    radioDelay = par("radioDelay");
    pkLenBits = &par("pkLenBits");
    ACKLenBits = &par("ACKLenBits");
    mode = par("mode");
    test_mode = par("test_mode");

    minHosts = par("minHosts");
    maxHosts = par("maxHosts");
    maxHosts_old = maxHosts;
    if(test_mode == 1)
        maxHosts = minHosts;
    HostStepSize = par("HostStepSize");

    RX_TX_switching_time = par("RX_TX_switching_time");
    RX_TX_switching_time /= 1000000;    //convert to s
    HostMinimumSensingTime = par("HostMinimumSensingTime");
    HostMinimumSensingTime /= 1000000;    //convert to s
    carrier_sense_duration = par("carrier_sense_duration");
    carrier_sense_duration /= 1000000;    //convert to s
    cs_enabled = par("cs_enabled");
    data_duration = pkLenBits->doubleValue() / txRate;
    ACK_duration = ACKLenBits->longValue() / txRate;
    delay_after_ack = par("delay_after_ack");
    if(delay_after_ack == 0)
        delay_after_ack = RX_TX_switching_time;
    else
        delay_after_ack /= 1000000;    //convert to s
    haltOnPacketNumberSent = par("haltOnPacketNumberSent");
    numberHostsPlotModes = par("numberHostsPlotModes");
    frame_overhead = par("frame_overhead");


    delay_counts = 0;
    ACK_duration = ACKLenBits->longValue() / txRate;
    ACK_currently_send = false;
    index = 0;
    //channelBusy = false;

    tx_time = new cDoubleHistogram();
    rx_time = new cDoubleHistogram();
    delay = new cDoubleHistogram();

    //Strawman configuration
    Strawman_max_repetition = par("Strawman_max_repetition").longValue();
    Strawman_max_contention_length = par("Strawman_max_contention_length");
    Strawman_max_contention_length /= 1000;
    Strawman_min_contention_length = par("Strawman_min_contention_length");
    Strawman_min_contention_length /= 1000;
    Strawman_min_contention_slot_number = 0;
    Strawman_max_contention_slots = par("Strawman_max_contention_slots");
    Strawman_contention_slot_length = Strawman_max_contention_length / Strawman_max_contention_slots;
    Strawman_CR_length = par("Strawman_CR_length").longValue();
    Strawman_CR_duration = Strawman_CR_length / txRate;
    Strawman_decision_length = par("Strawman_decision_length").longValue();
    Strawman_decision_duration = Strawman_decision_length / txRate;
    Strawman_waiting_for_contention = false;
    Strawman_cp_end = new cMessage("Strawman_cp_end");
    Strawman_not_timeout = new cMessage("Strawman_not_timeout");
    Strawman_contention_start_delay = par("Strawman_contention_start_delay");
    Strawman_contention_start_delay /= 1000; //conversion to seconds
    if(Strawman_contention_start_delay == 0)
        Strawman_contention_start_delay = delay_after_ack;

    //Bin-MAC configuration
    BTCR_query_length = par("BTCR_query_length").longValue();
    BTCR_query_duration = BTCR_query_length / txRate;
    BTCR_delay_after_query = par("BTCR_delay_after_query").doubleValue();
    BTCR_delay_after_query /= 1000;
    if(BTCR_delay_after_query == 0)
        BTCR_delay_after_query = delay_after_ack;
    BTCR_wait_duration = par("BTCR_wait_duration").doubleValue();
    BTCR_wait_duration /= 1000;
    BTCR_wait_duration = delay_after_ack + carrier_sense_duration;
    BTCR_query_timeout = new cMessage("BTCR_query_timeout");

    //proposed configuration
    Proposed_ACK_length = par("Proposed_ACK_length").longValue();
    Proposed_ACK_duration = ((double)Proposed_ACK_length) / txRate;
    Proposed_slot_number = par("Proposed_slot_number").longValue();
    Proposed_slot_timeout = new cMessage("Proposed_slot_timeout");
    Proposed_recovery_mode = par("Proposed_recovery_mode").longValue();
    Proposed_recovery_cycles_max = par("Proposed_recovery_cycles_max").longValue();
    Proposed_slot_duration = 2*delay_after_ack + data_duration + ACK_duration;
    Proposed_timeout_duration = delay_after_ack + carrier_sense_duration;   //minislot size
    Proposed_max_slot_duration = delay_after_ack + 128*8/txRate;   //128bytes (includes overhead), can be adjusted

    Proposed_slot_number_start = par("Proposed_slot_number_start").longValue();
    Proposed_slot_number_stop = par("Proposed_slot_number_stop").longValue();

    if(mode == 5)//BTCR (is based on proposed mode)
    {
        mode = 1;
        //test_mode = 0;
        Proposed_slot_number = 2;
        Proposed_BTCR_mode = true;
    }
    else
        Proposed_BTCR_mode = false;


    if(test_mode == 3)
        Proposed_slot_number = Proposed_slot_number_start;


    //calculate ACK length and other parameters
    double id_field_size = 2.0;    //bin-mac and s2TDMA ID field length
    double addr_802_15_4 = 4.0; //link layer addr length
    double PPDU = 6.0;      //802.15.4
    double MPDU = 5.0;      //without addr field
    double overhead = PPDU + MPDU;

    switch(mode){
    case 1:  //s2TDMA
    case 3: //BTCR
        Proposed_ACK_duration = (overhead + 2*id_field_size)*8 / txRate;
        break;
    case 2: //Strawman
        ACK_duration = (overhead + addr_802_15_4)*8 / txRate;
        Strawman_CR_duration = ACK_duration;
        Strawman_decision_duration = ACK_duration;
        break;
    case 4: //CSMA
        ACK_duration = (PPDU + MPDU)*8 / txRate;
        break;
    default: break;
    }

    //TODO change payload and keep header constant -----------------------------------------
    if(test_mode == 2) //diff packet sizes, constant n
    {
        test_packet_size_step = 8;
        test_packet_size_start = frame_overhead * 8;
        test_packet_size_end = (frame_overhead + 102) * 8;

        test_packet_size_current = test_packet_size_start;
        data_duration = test_packet_size_current / txRate;
    }


    //CSMA variables
    CSMA_backoff_time = par("CSMA_backoff_time").doubleValue();
    CSMA_backoff_time /= 1000000;
    CSMA_max_retransmissions = par("CSMA_max_retransmissions").longValue();
    CSMA_max_backoffs = par("CSMA_max_backoffs").longValue();
    CSMA_wc_min = par("CSMA_wc_min").longValue();
    CSMA_wc_max = par("CSMA_wc_max").longValue();


    //external interference and clock drift
    ClockDriftEnabled = par("ClockDriftEnabled");
    ClockDriftRangePercent = par("ClockDriftRangePercent");
    ClockDriftRangePercent /= 100;                                                  // conversion to percent
    ClockDriftPlotMode = par("ClockDriftPlotMode");
    ClockDriftPlotStepNumber = par("ClockDriftPlotStepNumber");
    ClockDriftPlotStepNumber++;
    ExternalInterferenceEnable = par("ExternalInterferenceEnable");
    ExternalInterferenceDutyCycle = par("ExternalInterferenceDutyCycle");
    ExternalInterferenceDutyCycle /= 100;
    ExternalInterferencePlotMode = par("ExternalInterferencePlotMode");
    ExternalInterferencePlotStepNumber = par("ExternalInterferencePlotStepNumber");
    if(ClockDriftEnabled == false) ClockDriftPlotMode = false;
    if(ExternalInterferenceEnable == false) ExternalInterferencePlotMode = false;
    if(ClockDriftPlotMode == true) ExternalInterferencePlotMode = false; //both modes are exclusive
    deadline_missed_counter = 0;
    ClockDriftPlotCurrentStepNumber = 0;
    ExternalInterferencePlotCurrentStepNumber = 0;

    //can be adjusted
    ExternalInterference_active_min = 7*8/txRate;   //7 bytes (empty packet)
    ExternalInterference_active_max = 64*8/txRate;  //64 bytes

    if(minHosts == -1)
        minHosts = maxHosts;
    if(minHosts > maxHosts)
        maxHosts = minHosts;
    currentActiveNodes = minHosts;


    endRxEvent = new cMessage("end-reception");
    restartEvent = new cMessage("restartEvent");
    ACKFinishedEvent = new cMessage("ACK Done");
    ACKStartEvent = new cMessage("ACK Start");
    ExternalInterferenceEvent = new cMessage("ExternalInterferenceEvent");
    simtime_event = new cMessage("simtime_event");

    //initialize storage
    for(int i = 0; i< 101; i++)
    {
        statsAll[i].min = 1000;
        statsAll[i].max = 0;
        statsAll[i].mean = 0;
        statsAll[i].stddev = 0;
        statsAll[i].packetSmallReceived = 0;
        statsAll[i].packetSmallLost = 0;
        statsAll[i].sequencesReceived = 0;
        statsAll[i].sequencesLost = 0;
        statsAll[i].sequencesTotal = 0;
        statsAll[i].numberOfPackets = 0;
        statsAll[i].numberOfNodes = 0;
        statsAll[i].packetsSkipped = 0;
    }
    storage_index = 0;
    storage_iteration_number = 0;
    this->initVariables();



    //periodic data pulling
    if(mode != 1) //all modes except proposed
        //restart_interval = (2*data_duration+Strawman_contention_slot_length+Strawman_decision_duration+Strawman_CR_duration+Strawman_contention_start_delay)*2*currentActiveNodes;
        restart_interval = 100;
    else
        restart_interval = 0;
    scheduleAt(simTime() + 0, restartEvent);
    restart_counter = 0;
    currentPaketCountPerSequence = 1;


    //////////////////////////////////
    //Clock drift
    //STATUS: not finished yet
    if(ClockDriftEnabled)
    {
        if(ClockDriftPlotMode)
        {
            currentActiveNodes = numberHostsPlotModes;
        }
    }

    ////////////////////////////////////
    //External Interference
    else if(ExternalInterferenceEnable) // && (ExternalInterferenceDutyCycle > 0))// || ExternalInterferencePlotMode))
    {
        //ExternalInterference_is_active = true;
        //ExternalInterference_active_time = uniform(ExternalInterference_active_min, ExternalInterference_active_max);
        //scheduleAt(simTime() + ExternalInterference_active_time, ExternalInterferenceEvent);
        if(ExternalInterferencePlotMode == false)
        {
            ExternalInterferenceCurrentDutyCycle = ExternalInterferenceDutyCycle;
            //random start
            scheduleAt(simTime() + uniform(0, 10*data_duration), ExternalInterferenceEvent);
        }
        else
        {
            //ExternalInterferencePlotMode active
            currentActiveNodes = numberHostsPlotModes;
            ExternalInterferenceCurrentDutyCycle = 0;
        }

        //ExternalInterference_active_time = data_duration; //should not matter, since duty cycle is off after start
    }


    if (ev.isGUI())
        getDisplayString().setTagArg("i2",0,"x_off");
}


void Server::handleMessage(cMessage *msg)
{
    char name[10] = {"ACK"};

/////////////////////////////////////////////////////////////////////////
//// External Interference
///////////////////////////////////////////////////////////////
    if(msg == ExternalInterferenceEvent)
    {
        if(ExternalInterferenceCurrentDutyCycle == 100)
        {
            //channelBusy = true;
            ExternalInterference_is_active = true;
            return;
        }

        //stop interference
        if(ExternalInterference_is_active == true)
        {
            ExternalInterference_is_active = false;
            EV << "Interference stop" << endl;
            //EV << "ExternalInterferenceCurrentDutyCycle: " << ExternalInterferenceCurrentDutyCycle << endl;
            ExternalInterference_idle_time = ExternalInterference_active_time * (1 / ExternalInterferenceCurrentDutyCycle - 1);
            //EV << "ExternalInterference_idle_time: " << ExternalInterference_idle_time << endl;

            //if(!ACK_currently_send && !receivingPaket)
            //    channelBusy = false;

            //cancelEvent(ExternalInterferenceEvent);
            scheduleAt(simTime() + ExternalInterference_idle_time, ExternalInterferenceEvent);
        }
        //start interference
        else
        {
            //channelBusy = true;
            ExternalInterference_is_active = true;
            ExternalInterference_active_time = uniform(ExternalInterference_active_min, ExternalInterference_active_max);
            EV << "Interference start (length: "<< ExternalInterference_active_time*1e3 << " ms)" << endl;
            interference_start_timestamp = simTime().dbl();
            interference_stop_timestamp = simTime().dbl() + ExternalInterference_active_time;  //end time of interference
            scheduleAt(simTime() + ExternalInterference_active_time, ExternalInterferenceEvent);


            //Strawman
            if(mode == 2)
            {
                //if interference lasts longer than longest contention packet, add it to contention packet length
                if(Strawman_cp_end->isScheduled() && (interference_stop_timestamp > Strawman_cp_end->getArrivalTime().dbl()) && sleep_mode_enabled == false)
                {
                    EV << "Server: interference prolonged CP packet (debug 2)" << endl;
                    Strawman_longest_cp = interference_stop_timestamp - Strawman_cp_start_timestamp;
                    Strawman_cp_stop_timestamp = interference_stop_timestamp;
                    cancelEvent(Strawman_cp_end);   //re-schedule event to end of longest pulse
                    scheduleAt(simTime()+Strawman_longest_cp+delay_after_ack, Strawman_cp_end);
                }

            }

            //if currently receiving data, mark this data as corrupt
            if(receivingPaket)
            {
                lastPacketGood = false;
            }
            if(ACK_currently_send)
            {
                ACK_NAK_corrupt = true;
            }

            if(mode == 1 || mode == 2 || mode == 3)  //proposed & Strawman & BTCR
            {
                //check if interference pulse is longer than remaining data
                if(endRxEvent->isScheduled() && (endRxEvent->getArrivalTime().dbl() < interference_stop_timestamp))
                {
                    EV << "Server: reschedule endRxEvent to end of interference." << endl;
                    //reschedule the end of data
                    cancelEvent(endRxEvent);
                    scheduleAt(interference_stop_timestamp, endRxEvent); //collision handling is done in endRxEvent
                }
                else if(mode == 1 && Proposed_slot_timeout->isScheduled() && (simTime() - slot_start_timestamp < carrier_sense_duration))
                {
                    //count interference as packet collision
                    interference_in_empty_slot_flag = true;
                    //check if interference is longer
                    if(Proposed_slot_timeout->getArrivalTime().dbl() < interference_stop_timestamp)
                    {
                        EV << "Server: reschedule Proposed_slot_timeout to end of interference." << endl;
                        //reschedule timeout to end of interference
                        cancelEvent(Proposed_slot_timeout);
                        scheduleAt(interference_stop_timestamp, Proposed_slot_timeout);
                    }
                }
            }
        }
    }
///////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////
//// Restart Event (Probe Message)
///////////////////////////////////////////////////////////////
    //wake up hosts to start contention
    if(msg == restartEvent)
    {
        Host *host;
        restart_counter++;

        if(restart_counter > haltOnPacketNumberSent)
        {
            restart_counter = 0;
            start_next_iteration();
            return;
        }

        EV << endl << "Server: restart with n= " << currentActiveNodes << " ---------------------------------------------" << endl << endl << endl;

        //reset some variables
        lastPacketGood = true;
        receivingPaket = false;
        ACK_currently_send = false;
        ACK_NAK_corrupt = false;
        sequencesTotal += currentActiveNodes; //each activated node should send one packet
        sleep_mode_enabled = false; //server is awake and ready to receive packets

        for(int i=0; i<maxHosts; i++)
        {
            receiveCounter[i] = false;
        }

        //BTCR
        BTCR_higher_id = maxHosts;
        BTCR_lower_id = ceil((double)BTCR_higher_id/2);
        BTCR_last_successful_id = maxHosts+1;
        BTCR_split_selected = 1;
        BTCR_split_count = 0;

        //proposed
        Proposed_current_slot = 1;
        Proposed_slot_number_current = Proposed_slot_number;
        Proposed_first_cycle = true;
        Proposed_slot_contains_single_ids = false;
        Proposed_recovery_cycle_count = 0;
        Proposed_send_recovery_ids = false;
        Proposed_recovery_slot = false;

        //Strawman
        Strawman_CR_repetition_counter = 0;
        Strawman_not_timeout_counter = 0;

        if (ev.isGUI())
        {
            char buf[64];
            sprintf(buf, "Wake up! host number: /%i\n", currentActiveNodes);
            bubble(buf);
        }

        //generate random id list without duplicates
        int n;
        bool check;
        for (int i=0; i<currentActiveNodes; i++)
        {
            do {
                //n = (int) (uniform(0, maxHosts-1)+0.99999);
                n = (int) uniform(0, (double)maxHosts-0.00001);
                check = true;
                for(int j=0; j<i;j++)
                {
                    if(host_id_list[j] == n)
                    {
                        check = false;
                        break;
                    }
                }
            } while(!check);
            host_id_list[i] = n;
        }

        //debug: remove
        /*host_id_list[0] = 0;
        host_id_list[1] = 1;
        host_id_list[2] = 2;
        host_id_list[3] = 3;*/

        //wake up some hosts
        for (SubmoduleIterator iter(getParentModule()); !iter.end(); iter++)
        {
            if (iter()->isName("host")) // if iter() is in the same vector as this module
            {
                host = check_and_cast<Host *>(iter());
                int index_ = host->getIndex();


                for (int i=0; i< currentActiveNodes; i++)
                {
                    if(index_ == host_id_list[i]) //if host is within the random list, then activate it
                    {
                        if(test_mode == 2)  //different packet size, same n
                        {
                            host->wakeup(data_duration);
                        }
                        else
                        {
                            host->wakeup(0);
                        }
                        break;
                    }
                }
            }
        }
        if(mode != 1) //all modes except proposed
            scheduleAt(simTime() + restart_interval, restartEvent);
        else if(Proposed_BTCR_mode == false)//proposed
        {
            //send ack to start TDMA cycle
            Proposed_first_cycle = true;
            Proposed_handle_collision();
            ACK_type = PROPOSED_ACK;
            ACK_host_id = 1000; //some invalid value
            scheduleAt(simTime(), ACKFinishedEvent);
        }
        return;
    } //end restart event
///////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////
//// ACK and other control messages
///////////////////////////////////////////////////////////////
    //START (just wait, actual ACK packet is sent at end of ACK_duration)
    if(msg == ACKStartEvent)
    {
        double ack_duration_;
        ACK_currently_send = true;  //indicator that receiver is currently sending data

        if(receivingPaket || ExternalInterference_is_active)
        {
            ACK_NAK_corrupt = true;
        }
        else
        {
            ACK_NAK_corrupt = false;
        }
        if(receivingPaket)
            lastPacketGood = false;

        //choose message length according to ACK_type
        switch(ACK_type){
        case ACK: ack_duration_ = ACK_duration; break;
        case Strawman_CR: ack_duration_ = Strawman_CR_duration; break;
        case Strawman_Not: ack_duration_ = Strawman_decision_duration; break;
        case BTCR_ACK: ack_duration_ = BTCR_query_duration; break;
        case BTCR_NAK: ack_duration_ = BTCR_query_duration; break;
        case PROPOSED_ACK: ack_duration_ = Proposed_ACK_duration; break;
        case PROPOSED_NAK: ack_duration_ = Proposed_ACK_duration; break;
        case PROPOSED_NEXT_SLOT:
                ack_duration_ = 1/1E6;
                break;
        case CSMA_ACK:
        case CSMA_NAK: ack_duration_ = ACK_duration; break;
        default: break;
        }

        //schedule end of message
        scheduleAt(simTime()+ack_duration_, ACKFinishedEvent);
        return;
    }


    //END of ACK
    if(msg == ACKFinishedEvent)
    {

        if(receivingPaket || ExternalInterference_is_active)
        {
            ACK_NAK_corrupt = true;
        }

        switch(ACK_type){
        case ACK:
            if(mode != 2)   //strawman only
                break;
            if(sleep_mode_enabled) return;  //cancel if going to sleep
            if(ACK_NAK_corrupt == false) sequencesReceived++;
            EV << "Server: sent ACK to node: "<< ACK_host_id << endl;
            //ACK can be also new CR message -> schedule notification message and cancel it if no contention message is received
            Strawman_longest_cp = -1;
            Strawman_cp_start_timestamp = simTime().dbl() + Strawman_contention_start_delay;
            Strawman_waiting_for_contention = true;
            //scheduleAt(simTime()+Strawman_contention_start_delay+Strawman_max_contention_length+delay_after_ack, Strawman_cp_end);
            scheduleAt(simTime()+Strawman_contention_start_delay+carrier_sense_duration+delay_after_ack, Strawman_cp_end);  //schedule timeout
            break;
        case Strawman_CR:
            Strawman_longest_cp = -1;
            Strawman_cp_start_timestamp = simTime().dbl() + Strawman_contention_start_delay;
            Strawman_waiting_for_contention = true;
            //scheduleAt(simTime()+Strawman_contention_start_delay+Strawman_max_contention_length+delay_after_ack, Strawman_cp_end);
            scheduleAt(simTime()+Strawman_contention_start_delay+carrier_sense_duration+delay_after_ack, Strawman_cp_end);  //schedule timeout
            break;
        case Strawman_Not:
            EV << "Server: choose notification length:" << Strawman_longest_cp << endl;
            Strawman_waiting_for_contention = false;
            scheduleAt(simTime()+delay_after_ack+1e-6, Strawman_not_timeout);    //schedule timeout
            break;
        case BTCR_ACK:
            if(ACK_NAK_corrupt == false)
                sequencesReceived++;
            EV << "Server: successfully sent BTCR_ACK to node: "<< ACK_host_id << endl;
            if(sleep_mode_enabled == false)
                scheduleAt(simTime()+BTCR_wait_duration, BTCR_query_timeout);
            break;
        case BTCR_NAK:
            EV << "Server: sent BTCR_NAK " << endl;
            scheduleAt(simTime()+BTCR_wait_duration, BTCR_query_timeout);
            break;
        case CSMA_ACK:
            if(ACK_NAK_corrupt == false)
                sequencesReceived++;
            break;
        case PROPOSED_ACK:
            EV << "Server: sent PROPOSED_ACK";
            if(ACK_host_id == -1) EV <<" to end empty slot" << endl;
            else EV <<" with id: " << ACK_host_id << endl;
            //if((ACK_NAK_corrupt == false) && (ACK_host_id >= 0))
            if(ACK_host_id >= 0)
            {
                //sequencesReceived++;
                receiveCounter[ACK_host_id] = true;
                EV << "Server: sequencesReceived++" << endl;
            }
            break;
        case PROPOSED_NAK:
            EV << "Server: sent PROPOSED_NAK " << endl;
            //scheduleAt(simTime()+Proposed_slot_duration, Proposed_slot_timeout);
            break;
        case PROPOSED_NEXT_SLOT:
            EV << "Server: sent PROPOSED_NEXT_SLOT (slot empty)" << endl;
            break;
        case FORCE_SLEEP:
            sendACKPacket(name, ACK_host_id, ACK_type);
            return;
        default: break;
        }

        //BTCR
        if(mode == 3)
        {
            if(sleep_mode_enabled)
                BTCR_finish_cycle();
            else
                EV << "Server: BTCR_higher_id= " << BTCR_higher_id << "   BTCR_lower_id= " << BTCR_lower_id << "   BTCR_split_selected: " << BTCR_split_selected << endl;
        }

        //proposed
        if(mode == 1)
        {
            if(Proposed_send_recovery_ids)
                EV << "Recovery slot starts with range: higher_id = " << maxHosts-1 << "  lower_id = " << Proposed_slot_list[Proposed_current_slot].higher + 1 << endl;

            if(Proposed_recovery_slot)
                Proposed_recovery_slot = false;

            //last ID reached (cycle finished)
            if(((Proposed_current_slot > Proposed_slot_number_current) && (Proposed_lower_id >= 0)) || (Proposed_higher_id < 0))//last ID reached
            {
                EV << "debug 11" << endl;
                //recovery cycle
                if(mode == 1 && Proposed_recovery_mode == 2 && Proposed_recovery_cycle_count < Proposed_recovery_cycles_max)
                {
                    EV << "Server: started recovery cycle, count = " << Proposed_recovery_cycle_count << endl;
                    Proposed_recovery_cycle_count++;
                    Proposed_higher_id = maxHosts-1;
                    Proposed_lower_id = 0;
                    Proposed_generate_slots();
                    scheduleAt(simTime() + Proposed_timeout_duration, Proposed_slot_timeout);
                }
                else
                    Proposed_finish_cycle();
            }
            else
            {
                if(Proposed_send_recovery_ids == false)
                    EV << "Server: higher_id= " << Proposed_higher_id << "   lower_id= " << Proposed_lower_id << "   current_slot: " << Proposed_current_slot
                        << "/" << Proposed_slot_number_current << "  ID range: " << Proposed_slot_list[Proposed_current_slot].higher << "-" << Proposed_slot_list[Proposed_current_slot].lower << endl;
                scheduleAt(simTime() + Proposed_timeout_duration, Proposed_slot_timeout);
                //EV << "Server: schedule timeout at time:" << simTime() << "  to trigger at time: " << simTime() + Proposed_timeout_duration << endl;
            }
        }

        //GUI
        char buf[64];
        if (ev.isGUI())
        {
            sprintf(buf, "Message complete. ID%i\n", ACK_host_id);
            bubble(buf);
        }

        //reset some variables
        ACK_currently_send = false;
        slot_start_timestamp = simTime().dbl() + delay_after_ack; //timestamp for slot start

        //send ACK/NAK/ETC message
        sendACKPacket(name, ACK_host_id, ACK_type);
        return;
    } //// end ACKFinishedEvent


    //done a split, but no response --> increase split counter
    if(msg == BTCR_query_timeout)
    {
        BTCR_split_selected++;
        if(BTCR_split_selected > 2 || BTCR_lower_id == 0)
        {
            //sleep mode
            EV << "Server: BTCR did not receive reply (last ID split empty) -> enter sleep mode" << endl;
            sleep_mode_enabled = true;
            ACK_type = FORCE_SLEEP;
            scheduleAt(simTime()+delay_after_ack, ACKFinishedEvent);
            //BTCR_finish_cycle();
            return;
        }
        else
        {
            //calculate split (select lower half of interval)
            BTCR_higher_id = BTCR_lower_id-1;
            BTCR_lower_id = 0;
        }
        scheduleAt(simTime()+RX_TX_switching_time, ACKStartEvent);
    }


    //used to check if slot is empty
    if(msg == Proposed_slot_timeout)
    {
        //debug remove (simulate external interference in empty slot and therefore force slot splitting)
        //interference_in_empty_slot_flag = true;

        //slot was not empty, but interference occurred
        if(interference_in_empty_slot_flag || ExternalInterference_is_active)
        {
            EV << "debug: interference in empty slot" << endl;
            interference_in_empty_slot_flag = false;
            ACK_type = PROPOSED_NAK;
            Proposed_first_cycle = false; //needed for slot splitting function
            Proposed_handle_collision();
        }
        else
        {
            EV << "debug: empty slot" << endl;
            //slot is empty -> inform hosts that next slot starts
            Proposed_single_slot_repetition_counter = 0;
            Proposed_current_slot++;
            if((Proposed_current_slot > Proposed_slot_number_current) && (Proposed_lower_id > 0))    //sub-cycle finished
            {
                //BTCR mode start single slot, not cycle with multiple slots (only if only 2 IDs are remaining
                if(Proposed_BTCR_mode == true && (Proposed_lower_id>2))
                {
                    EV << "Server: BTCR: sub-cycle finished, create new slot" << endl;
                    Proposed_slot_number_current = 1;
                    Proposed_current_slot = 1;
                    Proposed_higher_id = Proposed_lower_id-1;
                    Proposed_lower_id = 0;
                    Proposed_slot_list[1].higher = Proposed_higher_id;
                    Proposed_slot_list[1].lower = Proposed_lower_id;
                }
                else
                {
                    //start next TDMA cycle
                    EV << "Server: last (sub-)slot was empty -> start new cycle" << endl;
                    Proposed_higher_id = Proposed_lower_id-1;
                    Proposed_lower_id = 0;
                    Proposed_generate_slots();
                }
            }
            //set id to out-of-range to indicate that slot was empty
            ACK_type = PROPOSED_ACK;
        }

        ACK_host_id = -1; //set the ID to an invalid value to indicate an empty slot
        scheduleAt(simTime() + delay_after_ack, ACKStartEvent);
    }

    //notification timeout: did not receive a packet after notification was sent (happens only when there is external interference)
    if(msg == Strawman_not_timeout)
    {
        Strawman_not_timeout_counter++;
        if(Strawman_not_timeout_counter < 2)
        {
            EV << "Server: Strawman notification timeout -> send CR" << endl;
            ACK_type = Strawman_CR;
            if(ExternalInterference_is_active)
            {
                //send contention request after interference_end + delay
                scheduleAt(interference_stop_timestamp+delay_after_ack, ACKStartEvent);//TODO check
            }
            else
            {
                //send contention request after delay_after_ack time
                scheduleAt(simTime()+delay_after_ack, ACKStartEvent);
            }
        }
        else    //limit reached, abort
        {
            EV << "Server: did not receive any reply a second time -> stop." << endl;
            Strawman_finish_cycle();
            return;
        }
    }


    //start sending notification message
    if(msg == Strawman_cp_end)
    {
        //EV << "Server: cp end event Strawman_longest_cp: " << Strawman_longest_cp << endl;
        //no contention packet (CP) received -> do not send notification
        if(Strawman_longest_cp == -1)
        {
            Strawman_CR_repetition_counter++;
            if(Strawman_CR_repetition_counter > 1)
            {
                EV << "Server: did not receive cp message a second time -> stop." << endl;
                Strawman_finish_cycle();
                return;
            }
            else
            {
                double delay_ = simTime().dbl()+RX_TX_switching_time;
                //shift CR to end of interference, or it will be lost again
                if(ExternalInterferenceEnable && ExternalInterference_is_active)    //TODO marker
                {
                    EV << "debug: interference_stop_timestamp: " << interference_stop_timestamp << endl;
                    delay_ = interference_stop_timestamp + delay_after_ack;
                }
                EV << "Server: no CP received -> repeat CR" << endl;
                ACK_type = Strawman_CR;
                scheduleAt(delay_, ACKStartEvent);
            }
            return;
        }
        //Strawman contention ended -> pick longest pulse and transmit decision
        ACK_type = Strawman_Not;
        scheduleAt(simTime()+Strawman_decision_duration, ACKFinishedEvent);
        ACK_currently_send = true;
    }
//////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//finished receiving, evaluate data
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (msg==endRxEvent)
    {
        packetSmallLost += currentCollisionNumFrames;
        currentCollisionNumFrames = 0;

        //EV << "Server: finished receiving packet(s)" << endl;

        ///////////////////////////////////////////////////////////////
        /////////successful reception //////////////////////////////////////////////////////////////////////////////
        if(lastPacketGood == true && ACK_currently_send== false)
        {
            if(mode == 1 && receiveCounter[lastID] == true) //proposed
            {
                //do not collect delay etc., if a packet from this node has already been received (only count the first received in case there are duplicates)
            }
            else
            {
                //collect statistics
                delay->collect(delay_temp);
                tx_time->collect(tx_time_temp);
                if(mode == 4)
                    rx_time->collect(rx_time_temp);     //CSMA already adds ACK to delay
                else
                    rx_time->collect(rx_time_temp + delay_after_ack + ACK_duration);   //add last ACK duration + switching and delay times (we count these as rx_time)
            }

            EV << "Server: successfully received message from node: " << lastID << " with delay: " << delay_temp << "  tx_time: " << tx_time_temp << "  rx_time: " << rx_time_temp << endl;

            ACK_host_id = lastID;

            //BCTR
            if(mode == 3)
            {
                BTCR_split_selected++;
                if(BTCR_split_selected > 2 || BTCR_lower_id == 0)
                {
                    //finish
                    EV << "Server: last split was successful -> send ACK and then sleep" << endl;
                    sleep_mode_enabled = true;
                    //BTCR_finish_cycle();
                }
                else
                {
                    //continue with next split-half
                    BTCR_higher_id = BTCR_lower_id - 1;
                    BTCR_lower_id = 0;
                }
            }


            //proposed
            if(mode == 1)
            {
                Proposed_slot_contains_single_ids = false;
                Proposed_current_slot++;    //last slot was successful -> continue with next one
                if(Proposed_current_slot > Proposed_slot_number_current)
                {
                    if(Proposed_lower_id > 0) //sub-cycle finished
                    {
                        //BTCR mode start single slot, not cycle with multiple slots (only if only 2 IDs are remaining
                        if(Proposed_BTCR_mode == true && (Proposed_lower_id>2))
                        {
                            EV << "Server: BTCR: sub-cycle finished, create new slot" << endl;
                            Proposed_slot_number_current = 1;
                            Proposed_current_slot = 1;
                            Proposed_higher_id = Proposed_lower_id-1;
                            Proposed_lower_id = 0;
                            Proposed_slot_list[1].higher = Proposed_higher_id;
                            Proposed_slot_list[1].lower = Proposed_lower_id;
                        }
                        else
                        {
                            //start next TDMA cycle
                            EV << "Server: sent ACK in last (sub)slot, start new TDMA cycle" << endl;
                            Proposed_higher_id = Proposed_lower_id-1;
                            Proposed_lower_id = 0;
                            Proposed_generate_slots();
                        }
                    }
                    else //cycle finished
                    {
                        //do nothing here (handling is done when transmitting the ACK)
                    }
                }
            }
        }//end: successful reception

        ///////////////////////////////////////////////////////////////
        ////////failed reception //////////////////////////////////////////////////////////////////////
        else
        {
            //BTCR
            if(mode == 3)
            {
                //calculate ID interval
                if(BTCR_split_count > 0)
                {
                    int BTCR_id_diff;
                    BTCR_id_diff = BTCR_higher_id - BTCR_lower_id;
                    BTCR_lower_id = BTCR_higher_id - BTCR_id_diff / 2;
                }
                BTCR_split_count++;
            }

            //Proposed
            if(mode==1)
            {
                //packet was corrupted -> start TDMA cycle
                EV << "Server: Proposed_handle_collision() called" << endl;
                Proposed_handle_collision();
            }
        }//end: failed reception


        switch(mode){
        case 1:  //proposed
            if(lastPacketGood) ACK_type = PROPOSED_ACK; else ACK_type = PROPOSED_NAK;
            break;
        case 2: //Strawman
            if(lastPacketGood) ACK_type = ACK; else ACK_type = Strawman_CR;
            break;
        case 3: if(lastPacketGood) ACK_type = BTCR_ACK; else ACK_type = BTCR_NAK; //BTCR
            break;
        case 4: if(lastPacketGood) ACK_type = CSMA_ACK; else ACK_type = CSMA_NAK; //CSMA
        break;
        default: break;
        }

        // update network graphics
        if (ev.isGUI())
        {
            getDisplayString().setTagArg("i2",0,"x_off");
            getDisplayString().setTagArg("t",0,"");
        }

        //CSMA
        //send ACK/NAK/CR
        if(mode == 4 && lastPacketGood == false)    //CSMA: do not send an ACK if packet was corrupt (node will timeout and then send again)
        {
            EV << "Server: packet(s) corrupt, send no ACK" << endl;
        }
        else if (ACKStartEvent->isScheduled() || ACKFinishedEvent->isScheduled())   //do not send if already scheduled
        {
            EV << "Server: ACK already scheduled -> do not schedule again" << endl;
        }
        else
            scheduleAt(simTime() + delay_after_ack, ACKStartEvent);

        //reset some variables
        lastPacketGood = true;
        receivingPaket = false;
        BTCR_split_selected = 1;

    } //if (msg==endRxEvent)



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//start receiving //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    else if(msg->isSelfMessage() == false)
    {
        if(sleep_mode_enabled)
            return;

        //strawman contention packet
        if(msg->hasPar("Strawman_cp_packet") == true)////////// Strawman contention pulse receive
        {
            cPacket *pkt = check_and_cast<cPacket *>(msg);
            double cp_length_;
            cp_length_ = pkt->par("cp_length");
            Strawman_CR_repetition_counter = 0;

            if(cp_length_ > Strawman_longest_cp)
            {
                Strawman_longest_cp = cp_length_;
                Strawman_cp_stop_timestamp = Strawman_cp_start_timestamp + Strawman_longest_cp;
                cancelEvent(Strawman_cp_end);   //reschedule event to end of longest pulse
                scheduleAt(simTime()+Strawman_longest_cp+delay_after_ack, Strawman_cp_end);
            }


            //TODO TEST
            //if interference is currently blocking the channel and interference would last longer than CP then re-schedule event
            if(ExternalInterferenceEnable && ExternalInterference_is_active && (Strawman_cp_stop_timestamp < interference_stop_timestamp))
            {
                bool longest_cp_;
                longest_cp_ = ceil((interference_stop_timestamp - Strawman_cp_start_timestamp) / Strawman_contention_slot_length) * Strawman_contention_slot_length;    //calculate cp with granularity of Strawman_contention_slot_length
                if(longest_cp_ > Strawman_max_contention_length)    //limit max value
                    longest_cp_ = Strawman_max_contention_length;

                cancelEvent(Strawman_cp_end);   //reschedule event to end of longest pulse
                scheduleAt(simTime()+longest_cp_+delay_after_ack, Strawman_cp_end);
            }

            //EV << "Server: receive cp packet with length " << cp_length_ << endl;
            Strawman_waiting_for_contention = true;

            delete pkt;
            return;
        }//end Strawman_cp_packet

        //BTCR
        if(mode == 3)
        {
            cancelEvent(BTCR_query_timeout);
        }

        //proposed
        if(mode == 1)
        {
            cancelEvent(Proposed_slot_timeout);
            cancelEvent(ACKStartEvent);
        }

        //Strawman
        if(mode == 2)
        {
            //received packet -> cancel notification timeout
            cancelEvent(Strawman_not_timeout);
            Strawman_not_timeout_counter = 0;
        }


        //read data from packet
        repetition = msg->par("repetition");
        id_temp = msg->par("id");
        delay_temp = msg->par("delay");
        tx_time_temp = msg->par("tx_time");
        rx_time_temp = msg->par("rx_time");

        //EV << "Server: start receiving from node: " << id_temp << "  delay_temp: " << delay_temp << "  tx_time_temp: " << tx_time_temp << "  rx_time_temp: " << rx_time_temp << endl;

        senderACKRequest = true;
        cPacket *pkt = check_and_cast<cPacket *>(msg);
        ASSERT(pkt->isReceptionStart());

        simtime_t endReceptionTime = simTime() + pkt->getDuration();
        packetsSkippedTotal++;

        //EV << "Server: start receiving packet from node: " << id_temp << endl;


        //normal operation: received just 1 packet, no collision
        if (receivingPaket == false && ACK_currently_send == false && ExternalInterference_is_active == false)
        {

            //recall handleMessage with message end endRxEvent
            //EV << "receive: normal operation so far" << endl;
            cancelEvent(endRxEvent);
            scheduleAt(endReceptionTime, endRxEvent);

            if (ev.isGUI())
            {
                getDisplayString().setTagArg("i2",0,"x_yellow");
                getDisplayString().setTagArg("t",0,"RECEIVE");
                getDisplayString().setTagArg("t",2,"#808000");

                //show packet number
                if(packetList[id_temp].packetsReceivedTotal < currentPaketCountPerSequence)
                {
                    char buf[32];
                    sprintf(buf, "ID %i  Repetition %i", id_temp, repetition);
                    bubble(buf);
                }
            }
        }


        //received multiple packets (collision)
        else
        {
            if (currentCollisionNumFrames == 0 && ACK_currently_send == false && ExternalInterference_is_active == false)
                currentCollisionNumFrames = 2;
            else if (ACK_currently_send == true) //new packet arrives during ACK transmission -> ACK gets lost and arriving message
            {
                ACKsLost++;
                currentCollisionNumFrames++;
                ACK_NAK_corrupt = true;
            }
            else // if deadline_missed == true or ExternalInterference_is_active
            {
                currentCollisionNumFrames++;
            }


            lastPacketGood = false;

            //when multiple packets collide, re-schedule endRxEvent to the end of the longer one
            if (endReceptionTime > endRxEvent->getArrivalTime())
            {
                //EV << "collision: reschedule" << endl;
                cancelEvent(endRxEvent);
                scheduleAt(endReceptionTime, endRxEvent);
            }
            //if interference is ongoing and would last longer than packet, then reschedule endRxEvent to end of interference   //TODO nicht auf ende von interference schedulen, sondern auf vielfaches von CCA
            if(mode == 1 || mode == 2 || mode == 3)  //proposed & Strawman & BTCR
            {
                //check if interference pulse is longer than remaining data
                if(endReceptionTime < interference_stop_timestamp)
                {
                    EV << "Server: reschedule endRxEvent to end of interference. (debug 2)" << endl;
                    //reschedule the end of data
                    cancelEvent(endRxEvent);
                    scheduleAt(interference_stop_timestamp, endRxEvent); //collision handling is done in endRxEvent
                }
            }

            // update network graphics
            char buf[32];
            if (ev.isGUI())
            {
                getDisplayString().setTagArg("i2",0,"x_red");
                getDisplayString().setTagArg("t",0,"COLLISION");
                getDisplayString().setTagArg("t",2,"#800000");
                sprintf(buf, "Collision! (%ld packets)", currentCollisionNumFrames);
                bubble(buf);
            }
            EV << buf << endl;
        }//end else: received multiple packets (collision)

        receivingPaket = true;
        //channelBusy = true;

        //inform_hosts_about_channel_state(true, true);
        delete pkt;
    }//end else: start receiving
    lastID = id_temp;

}

void Server::finish()
{
    double collisionSequences;


    //debug
    /*EV << "test output of finish()" << endl;
    EV << "avg receive time per sequence: " << rx_time->getMean() << "  min: "<< rx_time->getMin() << "   max: " << rx_time->getMax() << "   number of entries: " << rx_time->getCount() << endl;
    EV << "avg transmit time per sequence: " << tx_time->getMean() << "  min: "<< tx_time->getMin() << "   max: " << tx_time->getMax() << "   number of entries: " << rx_time->getCount() << endl;
    EV << "avg delay: " << delay->getMean() << "   number of entries: " << delay->getCount() << endl;

    return;*/


    std::ofstream myfile;
    std::string file_output = vectorOutput.substr(1,vectorOutput.size()-2); //remove quotation marks
    myfile.open (file_output.c_str(), std::ios::trunc);
    if(myfile.is_open() == false)
    {
        EV << "cannot open output log file!" << endl;
    }

    switch (mode)
    {
    case 1: if(Proposed_BTCR_mode == false)
                myfile << "mode = Proposed   Proposed_recovery_mode = " << Proposed_recovery_mode;
            else
                myfile << "mode = BTCR (based on proposed) ";
            break;
    case 2: myfile << "mode = Strawman  "; break;
    case 3: myfile << "mode = BTCR  "; break;
    case 4: myfile << "mode = CSMA  "; break;
    }
    myfile << "\n";

    if(ClockDriftPlotMode == true)
    {
        myfile << " ClockDriftPlotMode:1   ClockDriftPlotDriftRange: " << ClockDriftRangePercent
                << "   ClockDriftPlotStepNumber: " << ClockDriftPlotStepNumber  << "\n";
        myfile << "number of nodes: " << statsAll[0].numberOfNodes << "   number of packets: " << statsAll[0].numberOfPackets;
    }
    else if(ExternalInterferencePlotMode)
    {
        myfile << "ExternalInterferencePlotMode:1   Duty Cycle: " << ExternalInterferenceDutyCycle << "   Step number: " <<
                ExternalInterferencePlotStepNumber << "\n";
        myfile << "number of nodes: " << statsAll[0].numberOfNodes << "   number of packets: " << statsAll[0].numberOfPackets;
    }

    myfile << "haltOnPacketNumberSent: " << haltOnPacketNumberSent << endl;

    if(ClockDriftPlotMode == true)
    {
        //not implemented yet
        return;
    }

    else if(ExternalInterferencePlotMode == true)
    {
        myfile << "duty cycle; data loss [%]; sequencesTotal; sequencesLost; delay avg [ms]; delay min [ms]; delay max [ms]; rx_time avg [ms]; tx_time avg[ms]; \n";

        //if(ClockDriftPlotCurrentStepNumber > 100) ClockDriftPlotCurrentStepNumber = 100;
        for (int i = 0; i<= ExternalInterferencePlotStepNumber; i++) //ClockDriftPlotStepNumber
        {
            double current_duty_cycle = i * (100 * ExternalInterferenceDutyCycle / ExternalInterferencePlotStepNumber);


            if(statsAll[i].sequencesReceived != 0)
                collisionSequences = (double)statsAll[i].sequencesLost / (double)statsAll[i].sequencesTotal*100 ;
            else
                collisionSequences = -1;


            myfile << current_duty_cycle << ";" << collisionSequences << ";"
                    << statsAll[i].sequencesTotal << ";" << statsAll[i].sequencesLost << ";"
                    << statsAll[i].delay_avg * 1000 << ";" << statsAll[i].delay_min * 1000 << ";" << statsAll[i].delay_max * 1000  << ";"
                    << statsAll[i].rx_time_avg * 1000 << ";"
                    << statsAll[i].tx_time_avg * 1000 << endl;


            EV << "mode: " << mode << endl;
            EV << "ExternalInterferencePlotMode enabled:   duty cycle: " << ClockDriftRangePercent*100 << "%    Step Number: "
                    << ExternalInterferencePlotStepNumber << "    current step:" << i << "     current duty cycle: " << current_duty_cycle << "% \n";

            EV << "number of nodes: " << statsAll[i].numberOfNodes << endl;
            EV << "number of received Sequences: " << statsAll[i].sequencesReceived << "   lost: " << statsAll[i].sequencesLost
                    << "  loss in percent: " << collisionSequences << "\n";
            EV << "avg delay [ms]: " << statsAll[i].delay_avg*1000 << endl;
            EV << "----------------------------------------------------------" << endl << endl;
        }

        myfile.close();
        delete hopCountStats;
        return;
    }

    if(test_mode == 1)
    {
        myfile << "test_mode = " << test_mode << endl;
        myfile << "Number of Nodes; maxHosts; loss [%]; sequencesReceived; sequencesLost; delay avg [ms]; delay min; delay max; rx_time avg [ms]; tx_time avg[ms]\n";
    }
    else if(test_mode == 2)
    {
        myfile << "test_mode = " << test_mode << endl;
        myfile << "Number of Nodes; payload [bytes]; loss [%]; sequencesReceived; sequencesLost; delay avg [ms]; delay min; delay max; rx_time avg [ms]; tx_time avg[ms]\n";
    }
    else if (test_mode == 3 && mode == 1)
    {
        myfile << "test_mode = " << test_mode << "  n=" << currentActiveNodes;
        if(ExternalInterferenceEnable)
            myfile << " interference=" << ExternalInterferenceCurrentDutyCycle;
        myfile << endl << "Number of Nodes; Number of slots; loss [%]; sequencesReceived; sequencesLost; delay avg [ms]; delay min; delay max; rx_time avg [ms]; tx_time avg[ms]\n";
    }
    else
        myfile << "Number of Nodes; loss [%]; sequencesReceived; sequencesLost; delay avg [ms]; delay min; delay max; rx_time avg [ms]; tx_time avg[ms]\n";

    //output all results to console
    for (int i = 0; i< storage_index; i++)
    {

        //stop outputting when last entry is over (number == 0)
        //if(statsAll[i].numberOfNodes == 0)
        //    break;


        collisionSequences = (double)statsAll[i].sequencesLost/(double)(statsAll[i].sequencesTotal);
        collisionSequences *= 100;
        myfile << statsAll[i].numberOfNodes << ";";
        if(test_mode == 1)
            myfile << statsAll[i].maxHosts << ";";
        if(test_mode == 2)
            myfile << ((double)statsAll[i].packet_size / 8) - frame_overhead << ";"; //15byte overhead (802.15.4 with 4bit addr)
        if(test_mode == 3 && mode == 1)
            myfile  << Proposed_slot_number_start+i << ";";

        myfile  << collisionSequences << ";"
                << statsAll[i].sequencesReceived << ";" << statsAll[i].sequencesLost << ";"
                << statsAll[i].delay_avg * 1000 << ";" << statsAll[i].delay_min * 1000 << ";" << statsAll[i].delay_max * 1000  << ";"
                << statsAll[i].rx_time_avg * 1000 << ";"
                << statsAll[i].tx_time_avg * 1000 << endl;



        //output value to console
        EV << "mode: " << mode <<endl;
        //EV << "Clock Drift; sequence loss [%]; packet loss [%]; avg delay [ms]; \n";

        EV << "number of nodes: " << statsAll[i].numberOfNodes << endl;
        if(test_mode == 1)
            EV << "maxHosts: " << statsAll[i].maxHosts << endl;
        EV << "number of received Sequences: " << statsAll[i].sequencesReceived << "   lost: " << statsAll[i].sequencesLost
                        << "  loss in percent: " << collisionSequences << "\n";
        EV << "avg receive time per sequence: " << statsAll[i].rx_time_avg * 1000 << "  min: "<< statsAll[i].rx_time_min * 1000 << "   max: " << statsAll[i].rx_time_max * 1000 << endl;
        EV << "avg transmit time per sequence: " << statsAll[i].tx_time_avg * 1000 << "  min: "<< statsAll[i].tx_time_min * 1000 << "   max: " << statsAll[i].tx_time_max * 1000 << endl;
        EV << "avg delay: " << statsAll[i].delay_avg << "  min: "<< statsAll[i].delay_min << "   max: " << statsAll[i].delay_max << endl;
        EV << "----------------------------------------------------------" << endl << endl;

    } //end: output all results to console: for (int i = 0; i< 101; i++)


    myfile.close();
    delete hopCountStats;
}


//increase number of nodes or number of packets for simulation
void Server::start_next_iteration()
{
    EV << "Server: start_next_iteration() called" << endl;

    //save data
    /*statsAll[storage_index].min = (int)hopCountStats->getMin();
    statsAll[storage_index].max = (int)hopCountStats->getMax();
    statsAll[storage_index].mean = (float)hopCountStats->getMean();
    statsAll[storage_index].stddev = (float)hopCountStats->getStddev();*/
    statsAll[storage_index].packetSmallReceived = packetSmallReceived;
    statsAll[storage_index].packetSmallLost = packetSmallLost;
    statsAll[storage_index].sequencesReceived = sequencesReceived;
    statsAll[storage_index].sequencesLost = sequencesTotal - sequencesReceived;
    statsAll[storage_index].sequencesTotal = sequencesTotal;
    statsAll[storage_index].numberOfPackets = currentPaketCountPerSequence;
    statsAll[storage_index].numberOfNodes = currentActiveNodes;
    statsAll[storage_index].maxHosts = maxHosts;
    statsAll[storage_index].packet_size = test_packet_size_current;
    statsAll[storage_index].ACKsLost = ACKsLost;
    statsAll[storage_index].packetsSkipped = packetsSkipped;
    statsAll[storage_index].deadline_missed_counter = deadline_missed_counter;

    statsAll[storage_index].delay_avg = delay->getMean();
    statsAll[storage_index].delay_min = delay->getMin();
    statsAll[storage_index].delay_max = delay->getMax();
    statsAll[storage_index].rx_time_avg = rx_time->getMean();
    statsAll[storage_index].rx_time_min = rx_time->getMin();
    statsAll[storage_index].rx_time_max = rx_time->getMax();
    statsAll[storage_index].tx_time_avg = tx_time->getMean();
    statsAll[storage_index].tx_time_min = tx_time->getMin();
    statsAll[storage_index].tx_time_max = tx_time->getMax();


    //debug: remove
    if(statsAll[storage_index].sequencesLost > 0)
    {
        EV << "Server: Lost packet!!!!!!!!!!!!" << endl;
    }

    //debug
    for(int i=0; i <= storage_index; i++)
    {
        EV << "statsAll["<< i << "].sequencesReceived " << statsAll[i].sequencesReceived << endl;
        EV << "statsAll["<< i << "].sequencesLost " << statsAll[i].sequencesLost << endl;
        EV << "statsAll["<< i << "].sequencesTotal " << statsAll[i].sequencesTotal << endl;
    }


    storage_index++;
    initVariables();


    //Configure settings for next iterations

    /////////////////////////////////////////////////////////////
    //Clock drift
    if(ClockDriftPlotMode == true)
    {
        ClockDriftPlotCurrentStepNumber++;

        if(ClockDriftPlotCurrentStepNumber >= ClockDriftPlotStepNumber+1)
        {
            this->endSimulation(); //finish()
            return;
        }

        //debug output
        EV << "clock drift iterator: " << ClockDriftPlotCurrentStepNumber << "   progress: " << ceil((double)ClockDriftPlotCurrentStepNumber/((double)ClockDriftPlotStepNumber+1)*100) << "%" << endl;
        std::cout << "clock drift iterator: " << ClockDriftPlotCurrentStepNumber << "   progress: " << ceil((double)ClockDriftPlotCurrentStepNumber/((double)ClockDriftPlotStepNumber+1)*100) << "%" << std::endl;
    }

    /////////////////////////////////////////////////////////////
    //external interference
    else if(ExternalInterferencePlotMode == true)
    {
        //increase duty cycle
        ExternalInterferencePlotCurrentStepNumber++;
        ExternalInterferenceCurrentDutyCycle = (ExternalInterferenceDutyCycle / ExternalInterferencePlotStepNumber) * ExternalInterferencePlotCurrentStepNumber;

        //restart events
        cancelEvent(ExternalInterferenceEvent);
        ExternalInterference_is_active = false;
        scheduleAt(simTime() + ExternalInterference_active_time, ExternalInterferenceEvent);

        if(ExternalInterferencePlotCurrentStepNumber >= ExternalInterferencePlotStepNumber+1)
        {
            this->endSimulation(); //finish()
            return;
        }

        //debug output
        EV << "external interference level: " << ExternalInterferenceCurrentDutyCycle << "   progress: " << ceil((double)ExternalInterferencePlotCurrentStepNumber/((double)ExternalInterferencePlotStepNumber+1)*100) << "%" << endl;
        std::cout << "external interference level: " << ExternalInterferenceCurrentDutyCycle << "   progress: " << ceil((double)ExternalInterferencePlotCurrentStepNumber/((double)ExternalInterferencePlotStepNumber+1)*100) << "%" << std::endl;
    }// end external interference

    /////////////////////////////////////////////////////////////
    //normal operation
    else if(test_mode == 1) //increase maxHosts, but currentNodes will stay constant at minHosts (useful for proposed and BTCR)
    {
        maxHosts += HostStepSize;
        if(maxHosts > maxHosts_old)
            this->endSimulation();
    }
    else if(test_mode == 2) //increase packet size (hosts will be informed in wakeup())
    {
        test_packet_size_current += test_packet_size_step;
        if(test_packet_size_current > test_packet_size_end)
            this->endSimulation();
        data_duration = test_packet_size_current / txRate;
        std::cout << "Server: increased packet size to: " << test_packet_size_current << "   progress: " << ceil((double)(test_packet_size_current-test_packet_size_step)/((double)test_packet_size_end)*100) << "%" << std::endl;
    }
    else if(test_mode == 3 && mode == 1) //increase slot numbers
    {
        Proposed_slot_number++;
        if(Proposed_slot_number > Proposed_slot_number_stop)
            this->endSimulation();

        EV << "Server: increased slot number to: " << Proposed_slot_number << endl;
        std::cout << "Server: increased slot number to: " << Proposed_slot_number << "   progress: " << ceil((double)(Proposed_slot_number-1)/((double)Proposed_slot_number_stop)*100) << "%" << std::endl;
    }
    else    //increase host number
    {
        currentActiveNodes += HostStepSize;
        if(currentActiveNodes > maxHosts)
            this->endSimulation();
        std::cout << "next iteration with n= " << currentActiveNodes << "   progress: " << ceil((double)(currentActiveNodes-HostStepSize)/((double)maxHosts)*100) << "%" << std::endl;
    }

    //if(mode == 2)
    //    restart_interval = (2*data_duration+Strawman_contention_slot_length+Strawman_decision_duration+Strawman_CR_duration+Strawman_contention_start_delay)*2*currentActiveNodes;

    cancelEvent(restartEvent);
    scheduleAt(simTime() + restart_interval, restartEvent);
}

void Server::initVariables()
{
    //packetList mit 0 initialisieren
    for(int i=0; i<currentActiveNodes+1; i++)
    {
        packetList[i].packetsReceivedTotal = 0;
        packetList[i].packetsReceivedGood = 0;
        packetList[i].firstPacketIndex = 1000;
        packetList[i].packetsSkipped = 0;

        nodesFinishedTransmission[i] = 1000;
    }

    lastID = 0;
    nodesFinishedTransmissionNumber = 0;
    sequencesReceived = 0;
    sequencesLost = 0;
    sequencesTotal = 0;
    packetSmallReceived = 0;
    packetSmallLost = 0;
    currentCollisionNumFrames = 0;
    ACKsLost = 0;
    packetsSkipped = 0;
    index = 0;
    packetsSkippedTotal = 0;
    delay_counts = 0;
    total_packets_transmitted = 0;
    total_packets_skipped = 0;
    total_acks_transmitted = 0;
    deadline_missed_counter = 0;
    lastPacketGood = true;
    receivingPaket = false;
    ACK_currently_send = false;
    slot_start_timestamp = 0;
    interference_in_empty_slot_flag = false;
    Proposed_slot_contains_single_ids = false;

    //reset storage containers
    rx_time->clearResult();
    tx_time->clearResult();
    delay->clearResult();

    //BTCR
    BTCR_higher_id = maxHosts;
    BTCR_lower_id = ceil((double)BTCR_higher_id/2);
    BTCR_last_successful_id = maxHosts+1;
    BTCR_split_selected = 1;
    BTCR_split_count = 0;

    //proposed
    Proposed_current_slot = 1;
    //Proposed_higher_id = maxHosts-1;
    //Proposed_lower_id = 0;
    Proposed_slot_number_current = Proposed_slot_number;
    Proposed_first_cycle = false;

    //debug
    EV << "Server: clearResults() called, rx_time now contains x= " << rx_time->getCount() << " elements" << endl;

    //ExternalInterference_is_active = false;
    //cancelEvent(ExternalInterferenceEvent);
    //channelBusy = false;
}



void Server::sendACKPacket(char *name, int host_id, int type)
{
    Host *host;
    cPacket *pk = new cPacket(name);
    pk->setBitLength(ACKLenBits->longValue());
    pk->addPar("type");
    pk->par("type").setLongValue((long) ACK_type);
    pk->addPar("host_id");
    pk->par("host_id").setLongValue(host_id);
    pk->addPar("Strawman_longest_cp");
    pk->par("Strawman_longest_cp").setDoubleValue(Strawman_longest_cp);
    pk->addPar("BTCR_higher_id");
    pk->par("BTCR_higher_id").setLongValue(BTCR_higher_id);
    pk->addPar("BTCR_lower_id");
    pk->par("BTCR_lower_id").setLongValue(BTCR_lower_id);
    pk->addPar("higher_id");
    pk->addPar("lower_id");
    if(mode == 1 && Proposed_recovery_mode == 1 && Proposed_send_recovery_ids)
    {
        //in recovery slot, send all processed IDs
        pk->par("higher_id").setLongValue(maxHosts-1);
        pk->par("lower_id").setLongValue(Proposed_slot_list[Proposed_current_slot].higher + 1);
        //Proposed_send_recovery_ids = false;
        Proposed_last_slot_was_recovery = true;
        EV << "sendACKPacket() recovery packet" << endl;
        Proposed_send_recovery_ids = false;
        Proposed_recovery_slot = true;  //by sending this ACK, the recovery cycle starts (gets reset when next ACK/NAK is sent)
        Proposed_current_slot--;
    }
    else
    {
        pk->par("higher_id").setLongValue(Proposed_slot_list[Proposed_current_slot].higher);
        pk->par("lower_id").setLongValue(Proposed_slot_list[Proposed_current_slot].lower);
    }
    pk->addPar("sleep");
    pk->par("sleep").setLongValue(sleep_mode_enabled);
    pk->addPar("corrupt");
    pk->par("corrupt").setBoolValue(ACK_NAK_corrupt);

    //EV << "sendACKPacket() called with higher_id= " << Proposed_slot_list[Proposed_current_slot].higher << "  and lower_id= " << Proposed_slot_list[Proposed_current_slot].lower << endl;

    //send packets
    for (SubmoduleIterator iter(getParentModule()); !iter.end(); iter++)
    {
        if (iter()->isName("host"))
        {
            host = check_and_cast<Host *>(iter());
            int index_ = host->getIndex();

            for(int i=0; i<currentActiveNodes; i++)
            {
                if(index_ == host_id_list[i])
                {
                    cPacket *copy = pk->dup();
                    sendDirect(copy, 0, 0, host->gate("in"));
                    break;
                }
            }
        }
    }
    delete pk;

}

//simple cs
bool Server::channel_busy2()
{
    Enter_Method("channel_busy2()");
    return (receivingPaket | ExternalInterference_is_active | ACK_currently_send);
}

//creates slot list with ID ranges
void Server::Proposed_generate_slots()
{
    int diff;
    Proposed_current_slot = 1;

    //check if enough IDs are left for splitting
    diff = (Proposed_higher_id - Proposed_lower_id + 1);
    if(diff < Proposed_slot_number)
    {
        EV << "Server: id range too small, limit slot number to: " << diff << endl;
        if(Proposed_slot_contains_single_ids == false)
        {
            EV << "Proposed_slot_contains_single_ids = true" << endl;
            Proposed_slot_contains_single_ids = true; //IDs now contain only 1 ID each
            Proposed_single_slot_repetition_counter = 0;
        }
        Proposed_slot_number_current = diff;

        for(int i=1; i<=Proposed_slot_number_current; i++)
        {
            Proposed_slot_list[i].higher = Proposed_higher_id - (i-1);
            Proposed_slot_list[i].lower = Proposed_slot_list[i].higher;
        }

        for(int i=1; i<=Proposed_slot_number_current; i++)
        {
            //debug: remove
            EV << "debug: slot number: " << i << "  higher: " << Proposed_slot_list[i].higher << "  lower: " << Proposed_slot_list[i].lower << endl;
        }

        return;
    }
    Proposed_slot_contains_single_ids = false;

    //calculate id ranges
    Proposed_slot_number_current = Proposed_slot_number;
    diff = (Proposed_higher_id - Proposed_lower_id + 1) / Proposed_slot_number_current;
    for(int i=1; i<Proposed_slot_number_current; i++)
    {
        Proposed_slot_list[i].higher = Proposed_higher_id - (i-1) * diff;
        Proposed_slot_list[i].lower = Proposed_slot_list[i].higher - diff+1;
    }

    //last slot
    Proposed_slot_list[Proposed_slot_number_current].higher = Proposed_slot_list[Proposed_slot_number_current - 1].lower - 1;
    Proposed_slot_list[Proposed_slot_number_current].lower = Proposed_lower_id;

    if(Proposed_lower_id == 0)
        Proposed_slot_list[Proposed_slot_number_current].lower = 0;

    for(int i=1; i<=Proposed_slot_number_current; i++)
    {
        //debug: remove
        EV << "debug: slot number: " << i << "  higher: " << Proposed_slot_list[i].higher << "  lower: " << Proposed_slot_list[i].lower << endl;
    }

}

//splits slots into slots with smaller ID ranges
//called when there was a collision
void Server::Proposed_handle_collision()
{
    //if collision happened within recovery slot, change ID ranges
    if(mode == 1 && Proposed_recovery_mode == 1)
    {
        if(Proposed_recovery_slot)
        {
            EV << "debug: collision within recovery-slot -> re-adjust ID ranges" << endl;
            Proposed_lower_id = Proposed_higher_id;
            Proposed_higher_id = maxHosts-1;
        }
        else if (Proposed_first_cycle==false)//(Proposed_higher_id < maxHosts-1)
        {
            EV << "Proposed_send_recovery_ids = true;" << endl;
            Proposed_send_recovery_ids = true;
        }
        else
        {
            EV << "debug: should not happen" << endl;
        }

    }

    //Slots have been split down to 1 ID already -> repeat them directly
    if(Proposed_slot_contains_single_ids == true)
    {
        //slot has been repeated twice already -> skip slot and continue with next ID and new cycle
        if(Proposed_single_slot_repetition_counter >= 2) //two retries
        {
            Proposed_single_slot_repetition_counter = 0;
            EV << "Server: skip slot after too many retries" << endl;

            //skip slot and check if sub-cycle or whole cycle have finished
            Proposed_current_slot++;
            if(Proposed_current_slot > Proposed_slot_number_current)
            {
                if(Proposed_lower_id > 0)   //sub-cycle finished
                {
                    //start next TDMA cycle
                    EV << "Server: last slot of sub-cycle was skipped -> start new cycle" << endl;
                    Proposed_higher_id = Proposed_lower_id-1;
                    Proposed_lower_id = 0;
                    Proposed_generate_slots();
                }
                else    //finished all IDs
                {
                    //do nothing, since cycle will finish in ACKFinishedEvent
                }
            }
        } // end: after 2 retries

        else //repeat slot
        {
            Proposed_single_slot_repetition_counter++;
            EV << "Server: repeat slot (retry " << Proposed_single_slot_repetition_counter << ")" << endl;
            //Proposed_higher_id = Proposed_slot_list[Proposed_current_slot].higher;
            //Proposed_lower_id = Proposed_higher_id;
            //Proposed_generate_slots();
        }
        return;
    }

    //check if TDMA is already in progress -> yes: create sub-slots -> no: create basic TDMA with full ID range
    else if (Proposed_first_cycle == true)//first collision-> create a new TDMA cycle
    {
        EV << "Server: start first TDMA cycle" << endl;
        Proposed_higher_id = maxHosts-1;
        Proposed_lower_id = 0;
        Proposed_first_cycle = false;
        Proposed_generate_slots();
    }
    else  //collision occurred in TDMA slot -> create sub-slots
    {
        EV << "Server: collision occurred -> split slots" << endl;
        Proposed_higher_id = Proposed_slot_list[Proposed_current_slot].higher;
        Proposed_lower_id = Proposed_slot_list[Proposed_current_slot].lower;
        Proposed_generate_slots();
    }

}

void Server::Proposed_finish_cycle()
{
    //cancel events
    cancelEvent(Proposed_slot_timeout);
    cancelEvent(endRxEvent);
    cancelEvent(ACKStartEvent);
    cancelEvent(ACKFinishedEvent);

    /*//invalidate ID ranges to avoid that nodes re-join at last ACK/NAK
    Proposed_higher_id = -1;
    Proposed_lower_id = -1;
    for(int i=0; i<=Proposed_slot_number+1; i++)
    {Proposed_slot_list[Proposed_current_slot].higher = -1;
    Proposed_slot_list[Proposed_current_slot].lower = -1;
        Proposed_slot_list[i].higher = -1;
        Proposed_slot_list[i].lower = -1;
    }*/
    sleep_mode_enabled = true;  //force other nodes to go sleeping as well (simulates a timeout, since server is not listening anymore)
    ACK_type = PROPOSED_ACK;

    //count received sequences
    for(int i=0; i<currentActiveNodes; i++)
    {
        if(receiveCounter[host_id_list[i]] == true)
            sequencesReceived++;
    }

    EV << "Server: finished cycle. Going to sleep." << endl;
    //EV << "iteration number: " << restart_counter << endl;
    //EV << "sequencesTotal " << sequencesTotal << "   sequencesReceived " << sequencesReceived << endl;

    /*if(sequencesTotal > sequencesReceived)
    {
        EV << "Server: lost packet here" << endl;
    }*/

    scheduleAt(simTime() + 0.1, restartEvent);
}

void Server::Strawman_finish_cycle()
{
    cancelEvent(Strawman_cp_end);
    cancelEvent(endRxEvent);
    cancelEvent(ACKStartEvent);
    cancelEvent(ACKFinishedEvent);

    EV << "Server: Going to sleep." << endl;

    sleep_mode_enabled = true;
    ACK_type = FORCE_SLEEP;
    scheduleAt(simTime(), ACKFinishedEvent);    //transmit packet to put all nodes to sleep
    cancelEvent(restartEvent);
    scheduleAt(simTime() + 1, restartEvent);
}

void Server::BTCR_finish_cycle()
{
    cancelEvent(BTCR_query_timeout);
    cancelEvent(endRxEvent);
    cancelEvent(ACKStartEvent);
    cancelEvent(ACKFinishedEvent);

    EV << "Server: Going to sleep." << endl;

    sleep_mode_enabled = true;

    cancelEvent(restartEvent);
    scheduleAt(simTime() + 1, restartEvent);
}

}; //namespace
