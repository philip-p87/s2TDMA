//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//


#include "Host.h"
#include "Server.h"

#include <string>
#include <stdlib.h>
#include <iomanip>


namespace aloha {

Define_Module(Host);

Host::Host()
{
}


Host::~Host()
{
    cancelAndDelete(endTxEvent);
    cancelAndDelete(endRxEvent);
    cancelAndDelete(endCarrierSenseEvent);
    cancelAndDelete(timeoutEvent);
}


void Host::initialize()
{
    stateSignal = registerSignal("state");
    server = simulation.getModuleByPath("server");
    if (!server) error("server not found");

    //get variables
    txRate = par("txRate");
    radioDelay = par("radioDelay");
    pkLenBits = &par("pkLenBits");
    ACKLenBits = &par("ACKLenBits");
    mode = par("mode");

    minHosts = par("minHosts");
    maxHosts = par("maxHosts");
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
    Strawman_contention_start_delay = par("Strawman_contention_start_delay");
    Strawman_contention_start_delay /= 1000;

    if(Strawman_contention_start_delay == 0)
        Strawman_contention_start_delay = delay_after_ack;

    //Bin-MAC configuration
    BTCR_query_length = par("BTCR_query_length").longValue();
    BTCR_delay_after_query = par("BTCR_delay_after_query").doubleValue();
    BTCR_delay_after_query /= 1000;
    if(BTCR_delay_after_query == 0)
        BTCR_delay_after_query = delay_after_ack;
    BTCR_wait_duration = par("BTCR_wait_duration").doubleValue();

    //proposed configuration
    Proposed_ACK_length = par("Proposed_ACK_length").longValue();
    Proposed_ACK_duration = ((double)Proposed_ACK_length) / txRate;
    Proposed_slot_number = par("Proposed_slot_number").longValue();
    Proposed_recovery_mode = par("Proposed_recovery_mode").longValue();

    //CSMA variables
    CSMA_backoff_time = par("CSMA_backoff_time").doubleValue();
    CSMA_backoff_time /= 1000000;
    CSMA_max_retransmissions = par("CSMA_max_retransmissions").longValue();
    CSMA_max_backoffs = par("CSMA_max_backoffs").longValue();
    CSMA_wc_min = par("CSMA_wc_min").longValue();
    CSMA_wc_max = par("CSMA_wc_max").longValue();
    CSMA_min_BE = par("CSMA_min_BE").longValue();
    CSMA_max_BE = par("CSMA_max_BE").longValue();


    if(mode == 5)//BTCR (is based on proposed mode)
    {
        mode = 1;
        Proposed_slot_number = 2;
        Proposed_BTCR_mode = true;
    }
    else
        Proposed_BTCR_mode = false;



    //external interference and clock drift
    ClockDriftEnabled = par("ClockDriftEnabled");
    ClockDriftRangePercent = par("ClockDriftRangePercent");
    ClockDriftRangePercent /= 100;                                                  // conversion to percent
    ClockDriftPlotMode = par("ClockDriftPlotMode");
    ClockDriftPlotStepNumber = par("ClockDriftPlotStepNumber");
    //ClockDriftPlotStepNumber++;
    ExternalInterferenceEnable = par("ExternalInterferenceEnable");
    ExternalInterferenceDutyCycle = par("ExternalInterferenceDutyCycle");
    ExternalInterferencePlotMode = par("ExternalInterferencePlotMode");
    ExternalInterferencePlotStepNumber = par("ExternalInterferencePlotStepNumber");


    if(ClockDriftEnabled == false) ClockDriftPlotMode = false;
    if(ExternalInterferenceEnable == false) ExternalInterferencePlotMode = false;
    if(ClockDriftPlotMode == true) ExternalInterferencePlotMode = false; //both modes are exclusive
    ClockDriftPlotCurrentStepNumber = 0;


    if(minHosts == -1)
        minHosts = maxHosts;
    current_number_of_active_hosts = minHosts;



    packetNumberMax = 1;
    transmissions_successful = 0;
    transmissions_failed = 0;



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



    //TODO move to .ini file
    timeout_duration = 1;//Strawman_max_contention_length * 2; //8ms    //TODO eventuell für strawman noch implementieren
    if(mode == 4)
    {
        timeout_duration = delay_after_ack + ACK_duration + 1e-6; //shortly after ack finished
    }




    //initialize variables
    reset_variables();
    state = SLEEP;
    endRxEvent = new cMessage("endRxEvent");
    endCarrierSenseEvent = new cMessage("endCarrierSense");
    endTxEvent = new cMessage("send/endTx");
    timeoutEvent = new cMessage("timeoutEvent");


    if (ev.isGUI())
        getDisplayString().setTagArg("t",2,"#808000");


    //start transmission at random time
    /*if(getIndex() < current_number_of_active_hosts)
    {
        scheduleAt(simTime() + uniform(0.1,0.2), endTxEvent);
    }*/

}

void Host::wakeup(double data_duration_ = 0)
{
    Enter_Method("wakeup()");

    double delay = 0;   //nodes are synchronized after preamble

    if(mode == 1 && Proposed_BTCR_mode == false) //only for proposed, not BTCR
        delay = delay_after_ack;

    //CSMA
    if(mode == 4) //start with back-off to avoid collisions
    {
        delay = CSMA_get_random_backoff();
        delay_time_stamp = simTime();   //count initial delay (first back-off)
    }
    else
        delay_time_stamp = simTime()+delay;

    //test_mode 2: diff packet sizes -> update here (calculation of data_duration_ done in server.cc)
    if(data_duration_ != 0)
    {
        data_duration = data_duration_;
    }

    reset_variables();
    state = IDLE;

    if (ev.isGUI())
    {
        getDisplayString().setTagArg("i",1,"white");
        getDisplayString().setTagArg("t",0,"waked up");
    }

    /*//add initial delay for first collision for strawman
    if(current_number_of_active_hosts > 1)
        rx_time = delay;*/

    //start sending with an offset if defined
    if(mode != 1 || Proposed_BTCR_mode == true) //all modes except proposed (this is started by sink)
        scheduleAt(simTime()+delay, endTxEvent);
}
void Host::sleep()
{
    Enter_Method("sleep()");
    char buf[64];

    if (ev.isGUI())
    {
        getDisplayString().setTagArg("i",1,"white");
        sprintf(buf, "Sleep");
        getDisplayString().setTagArg("t",0,buf);
    }

    cancelEvent(endTxEvent);
    cancelEvent(endRxEvent);
    cancelEvent(endCarrierSenseEvent);
    cancelEvent(timeoutEvent);
    state = SLEEP;
}

void Host::activate()
{
    Enter_Method("activate()");

    reset_variables();
    state = IDLE;

    //stop all events before restarting
    this->cancelEvent(endTxEvent);

    EV<< "node: "<< this->getIndex() << " activate() called" << endl;

    //start_at_random();
}

//change number of sending nodes
void Host::change_transmission_scheme(int numberOfTransmittingNodes)
{
    Enter_Method("change_transmission_scheme()");
    //cancel all running operations
    //this->cancelEvent(endTxEvent);
    //settings_changed_flag = true;

    EV<< "node: "<< this->getIndex() << " change_transmission_scheme() called with numberOfTransmittingNodes: " << numberOfTransmittingNodes << endl;

    //reset variables
    reset_variables();
    current_number_of_active_hosts = numberOfTransmittingNodes;
    state = IDLE;

    if(ClockDriftPlotMode == true)
        ClockDriftPlotCurrentStepNumber++;

}

void Host::stop_transmission()
{
    Enter_Method("stop_transmission()");
    cancelEvent(endTxEvent);
    cancelEvent(endCarrierSenseEvent);

    //reset variables
    reset_variables();
    state = IDLE;

    if (ev.isGUI())
    {
        getDisplayString().setTagArg("i",1,"");
        getDisplayString().setTagArg("t",0,"");
    }

    EV << "node: "<< this->getIndex() << " stop_transmission() called" << endl;
    EV << "node: "<< this->getIndex() << " current_number_of_active_hosts: " << current_number_of_active_hosts << endl;
}




void Host::handleMessage(cMessage *msg)
{
    char buf[32];
    double Strawman_cp_delay = delay_after_ack;
    double Strawman_not_delay = delay_after_ack;

    if(msg->isSelfMessage() == true) //message is a self-message
    {
        //packet message
        switch(mode){
        case 1: self_message_proposed(msg); break;      //proposed
        case 2: self_message_strawman(msg); break;      //Strawman
        case 3: self_message_btcr(msg); break;          //BTCR
        case 4: self_message_CSMA(msg); break;          //CSMA
        default: break;
        }
    } //end selfmessage

    //received ACK/NAK/etc message
    else if (msg->isName("ACK"))
    {
        //ignore ACK if in sleep mode
        if(state == SLEEP)
        {
            delete msg;
            return;
        }

        //read all parameters
        ACK_type = (ack_t) msg->par("type").longValue();
        int host_id = msg->par("host_id").longValue();
        ACK_NAK_corrupt = msg->par("corrupt").boolValue();
        //ACK_NAK_corrupt = false; // debug: remove
        int higher_id = (int)msg->par("higher_id").longValue();
        int lower_id = (int)msg->par("lower_id").longValue();
        bool server_is_sleeping = msg->par("sleep").boolValue();

        //when sink (server) is sleeping, then go to sleep mode as well
        //simulates a timeout, since there will be no more reply from the server as it is sleeping
        if(server_is_sleeping && !(mode == 3 &&  host_id == this->getIndex() && ACK_type == BTCR_ACK)
                && !(mode == 1 && host_id == this->getIndex() && ACK_type == PROPOSED_ACK))
        {
            EV << "node: "<< this->getIndex() << " server is sleeping, go to sleep mode as well" << endl;
            sleep();
            delete msg;
            return;
        }

        //int BTCR_higher_id = (int)msg->par("BTCR_higher_id").longValue();
        //int BTCR_lower_id = (int)msg->par("BTCR_lower_id").longValue();

        //received answer from server: cancel timeout
        cancelEvent(timeoutEvent);
        if(mode != 4)
        {
            cancelEvent(endTxEvent);
            cancelEvent(endCarrierSenseEvent);
        }


        //console debug output
        switch(ACK_type)
        {
        case ACK:
            EV << "node: " << this->getIndex() << "  received ACK";
            break;
        case Strawman_CR:
            EV << "node: " << this->getIndex() << "  received Strawman_CR";
            break;
        case Strawman_Not:
            EV << "node: " << this->getIndex() << "  received Strawman_Not";
            break;
        case BTCR_ACK:
            EV << "node: " << this->getIndex() << "  received BTCR_ACK";
            break;
        case BTCR_NAK:
            EV << "node: " << this->getIndex() << "  received BTCR_NAK";
            break;
        case CSMA_ACK:
            EV << "node: " << this->getIndex() << "  received CSMA_ACK";
            break;
        case CSMA_NAK:
            EV << "node: " << this->getIndex() << "  received CSMA_NAK";
            break;
        case PROPOSED_ACK:
            EV << "node: " << this->getIndex() << "  received PROPOSED_ACK";
            break;
        case PROPOSED_NAK:
            EV << "node: " << this->getIndex() << "  received PROPOSED_NAK";
            break;
        case PROPOSED_NEXT_SLOT:
            EV << "node: " << this->getIndex() << "  received PROPOSED_NEXT_SLOT";
            break;
        default: EV << "unknown: " << ACK_type << endl;
            break;
        }
        if(ExternalInterferenceEnable || ACK_NAK_corrupt)
        {
            if(ACK_NAK_corrupt)
                EV << " - corrupt" << endl;
            else
                EV << " - ok" << endl;
        }
        else
            EV << endl;



        //////////////////////////////////////////////////////////////////////
        ////Strawman resolution
        //////////////////////////////////////////////////////////////////////
        switch(ACK_type){
        case ACK:
            if(ACK_NAK_corrupt)
            {
                EV << "node: " << this->getIndex() << "  received corrupt ACK -> do nothing" << endl;
                break;
            }
            if(host_id == this->getIndex())
            {
                transmissions_successful++;
                //rxtime: since rx_time is attached to data, the last ACK will not be counted and must be added at server side
                EV << "node: " << this->getIndex() << "  successfully delivered message. Going to sleep." << endl;
                sleep();
            }
            else if(state == WAIT_FOR_NEXT_CONTENTION)
            {
                EV << "node: " << this->getIndex() << "  received ACK but treated as CR" << endl;
                //send next contention message
                state = CONTENTION;
                scheduleAt(simTime()+Strawman_contention_start_delay, endTxEvent);
            }
            break;
        case Strawman_CR:
            if(ACK_NAK_corrupt)
            {
                EV << "node: " << this->getIndex() << "  received corrupt Strawman_CR -> do nothing" << endl;
                break;
            }
            //check if transmission limit is reached    //TODO check
            if(packetNumber > Strawman_max_repetition)
            {
                 EV << "node: " << this->getIndex() << "  did not receive ACK. Packet limit reached. Going to SLEEP." << endl;
                 sleep();
            }
            else
            {
                Strawman_cp_active = true;
                state = CONTENTION;
                scheduleAt(simTime() + Strawman_cp_delay, endTxEvent);
            }
            break;
        case Strawman_Not:
            if(ACK_NAK_corrupt)
            {
                EV << "node: " << this->getIndex() << "  received corrupt Strawman_Not -> do nothing" << endl;
                break;
            }
            Strawman_cp_length_decision = msg->par("Strawman_longest_cp").doubleValue();
            //EV << "node: " << this->getIndex() << "  received notification with length: " << Strawman_cp_length_decision << endl;
            if (Strawman_cp_length_decision == Strawman_cp_length)
            {
                EV << "node: " << this->getIndex() << "  has been selected -> SEND" << endl;
                //node has been selected, re-transmit
                state = IDLE;
                scheduleAt(simTime()+Strawman_not_delay, endTxEvent);
            }
            else
            {
                EV << "node: " << this->getIndex() << "  has not been selected -> WAIT" << endl;
                //node has not been selected, wait until next CR message
                packetNumber = 1; //TODO check
                state = WAIT_FOR_NEXT_CONTENTION;

                /*double delay_buf;

                delay_buf = simTime().inUnit(-6);
                delay_buf /= 1E6;
                EV << "simTime():"  << simTime() << " converted to double: " << delay_buf << endl;*/

                //scheduleAt(simTime()+ t, timeoutEvent);
            }
            break;

        //////////////////////////////////////////////////////////////////////
        ////BTCR resolution
        //////////////////////////////////////////////////////////////////////
        case BTCR_ACK:
            if(host_id == this->getIndex())
            {
                transmissions_successful++;
                EV << "node: " << this->getIndex() << "  successfully delivered message. Going to sleep." << endl;
                sleep();
            }
            else    //ACK was not for this node
            {
                int BTCR_higher_id = msg->par("BTCR_higher_id").longValue();
                int BTCR_lower_id = msg->par("BTCR_lower_id").longValue();

                if(this->getIndex() <= BTCR_higher_id && this->getIndex() >= BTCR_lower_id)
                {
                    //selected -> transmit packet after short waiting time
                    EV << "node: " << this->getIndex() << "  was selected -> send" << endl;
                    state = IDLE;
                    scheduleAt(simTime()+BTCR_delay_after_query, endTxEvent);
                }
                else
                {
                    //EV << "node: " << this->getIndex() << "  Error: received ACK with complete range, but outside of range!!!!!!!!!!!!!!!!!" << endl;
                    EV << "node: " << this->getIndex() << "  was not selected -> wait" << endl;
                    state = WAIT_TO_GET_SELECTED;
                    scheduleAt(simTime()+timeout_duration, timeoutEvent);
                }
            }
            break;
        case BTCR_NAK:
            //collision occurred between multiple packets -> check if in new range and transmit if so
            int BTCR_higher_id, BTCR_lower_id;
            BTCR_higher_id = msg->par("BTCR_higher_id").longValue();
            BTCR_lower_id = msg->par("BTCR_lower_id").longValue();

            //selected -> transmit packet after short waiting time
            if(this->getIndex() <= BTCR_higher_id && this->getIndex() >= BTCR_lower_id)
            {
                state = IDLE;
                EV << "node: " << this->getIndex() << "  got selected in query." << endl;
                scheduleAt(simTime()+BTCR_delay_after_query, endTxEvent);
            }
            else
            {
                EV << "node: " << this->getIndex() << "  wait for next query." << endl;
                state = WAIT_TO_GET_SELECTED;
                scheduleAt(simTime()+timeout_duration, timeoutEvent);
            }
            break;

        //////////////////////////////////////////////////////////////////////
        //////Proposed scheme
        //////////////////////////////////////////////////////////////////////
        case PROPOSED_ACK:
        case PROPOSED_NAK:
        case PROPOSED_NEXT_SLOT:

            EV << "node: " << this->getIndex() << " received ID range: higher " << higher_id << " lower " << lower_id << endl;

            //check if control message was ok
            if(ACK_NAK_corrupt == true)
            {
                //start of desync
                if(desync == false)
                {
                    desync = true;
                    EV << "node: " << this->getIndex() << " desync: received corrupt ACK/NAK -> stay in rx mode and wait!"<< endl;
                }
                break;
            }
            else if(desync == true) //node was desynchronized and received good message -> re-sync
            {
                desync = false;
                if(higher_id < this->getIndex() && Proposed_recovery_mode == 0)
                {
                    //missed slot -> going to sleep
                    EV << "node: " << this->getIndex() << " resync, but missed slot -> going to sleep"<< endl;
                    sleep();
                }
                else
                    EV << "node: " << this->getIndex() << " resync!"<< endl;
            }

            //successful ACK -> sleep
            if(ACK_type == PROPOSED_ACK && host_id == this->getIndex())
            {
                transmissions_successful++;
                EV << "node: " << this->getIndex() << "  successfully delivered message. Going to sleep." << endl;
                sleep();
            }
            //for all control message types: transmit if ID is within range of upcoming slot
            else if(higher_id >= this->getIndex() && lower_id <= this->getIndex()) //check if next slot is within range
            {
                EV << "node: " << this->getIndex() << "  slot ID range matches own ID -> send packet" << endl;
                state = IDLE;
                scheduleAt(simTime() + delay_after_ack, endTxEvent);
            }
            //own ID is not within range of slot: do nothing
            else
            {
                EV << "node: " << this->getIndex() << "  ID range does NOT match -> do nothing" << endl;
            }
            break;

        //////////////////////////////////////////////////////////////////////
        /////////CSMA
        //////////////////////////////////////////////////////////////////////
        case CSMA_ACK:
            if(state != WAIT_FOR_REPLY) //node is not in receive mode
            {
                EV << "node: " << this->getIndex() << "  received ACK, but did not listen." << endl;
                break;
            }
            if(ACK_NAK_corrupt == true) //not used anymore: sink will not send an ACK if this would be corrupt -> host will timeout then
            {
                //received corrupt ACK -> handle as NAK
                EV << "node: " << this->getIndex() << "  received corrupt ACK, retry" << endl;
                CSMA_retry_number++;
                CSMA_retry(delay_after_ack);
            }
            else if(host_id == this->getIndex())
            {
                transmissions_successful++;
                EV << "node: " << this->getIndex() << "  successfully delivered message. Going to sleep." << endl;
                sleep();
            }
            else    //should not happen
            {
                EV << "node: " << this->getIndex() << "  received ACK with wrong ID. Ack host_id= " << host_id << endl;
                CSMA_retry_number++;
                CSMA_retry(delay_after_ack);
            }
            break;
        case CSMA_NAK:  //unused
            if(state == WAIT_FOR_REPLY)
            {
                EV << "node: " << this->getIndex() << "  received NAK, retry." << endl;
                CSMA_retry_number++;
                CSMA_retry(delay_after_ack);
            }
            break;
        case NAK:
            break;
        default: break;
        }


        // update network graphics
        if (ev.isGUI())
        {
            getDisplayString().setTagArg("i",1,"white");
            sprintf(buf, "Received ACK");
            getDisplayString().setTagArg("t",0,buf);
        }

        delete msg;
    }//end else if (msg->isName("ACK"))

}


void Host::self_message_proposed(cMessage *msg)
{
    if(state == IDLE)
    {
        //transmit packet
        packetNumber += 1;
        char packet_name_[64] = {"state==IDLE"};
        sendPacket(packet_name_, 0, 0);

        scheduleAt(simTime() + data_duration, endTxEvent);
        state = TRANSMIT_FINISHED;
    }
    else if(state == TRANSMIT_FINISHED)
    {
        //wait for receiver reply (ACK or NAK)
        //scheduleAt(simTime() + timeout_duration, timeoutEvent);
        state = WAIT_FOR_REPLY;
    }
    else if(state == WAIT_TO_GET_SELECTED)  //unused
    {
        //do nothing
        EV << "node: " << this->getIndex() << "  Error: did not get reply -> retry" << endl;
    }
}

void Host::self_message_strawman(cMessage *msg)
{
    //receiving went wrong -> receiver did not answer
    if (msg == timeoutEvent) {
        if(packetNumber > Strawman_max_repetition)
        {
            EV << "node: " << this->getIndex() << "  did not get reply -> abort" << endl;
            transmissions_failed++;
            sleep();
        }
        else //did not receive reply //TODO check
        {
            EV << "node: " << this->getIndex() << "  did not get reply -> wait" << endl;
            scheduleAt(simTime()+1, timeoutEvent);  //TODO check
            packetNumber++;

            //restart new packet after short delay
            //scheduleAt(simTime()+Strawman_contention_start_delay, endTxEvent);
            state = IDLE;
        }
    }

    else {
        if(state == IDLE)
        {
            //send packet
            char packet_name_[64] = {"state==IDLE"};
            sendPacket(packet_name_, 0, 0);
            scheduleAt(simTime() + data_duration, endTxEvent);
            state = TRANSMIT_FINISHED;
            packetNumber += 1;
        }
        else if(state == TRANSMIT_FINISHED)
        {
            //wait for receiver reply (ACK, CR or decision packet)
            //schedule timeout
            scheduleAt(simTime() + timeout_duration, timeoutEvent);
        }
        else if(state == CONTENTION)
        {
            //pick random contention packet length and send it delayed (tset + some additional time)
            int slot_number = (int) uniform(0, (double)Strawman_max_contention_slots+0.9999);
            Strawman_cp_length = slot_number * Strawman_contention_slot_length;

            //send contention message after some delay and wait for the rest of the cycle.
            //char packet_name_[64] = {"state==CONTENTION"};
            StrawmanSendContentionPacket(Strawman_cp_length);

            //sendPacket(packet_name_, Strawman_contention_start_delay, contention_length);

            //schedule end of time slot
            scheduleAt(simTime()+Strawman_max_contention_length, endTxEvent);
            state = TRANSMIT_FINISHED; //after contention message wait for reply (schedule timeout)
        }
        else if(state == WAIT_FOR_NEXT_CONTENTION)
        {
            //do nothing (sleep)
        }
    }
}

void Host::self_message_btcr(cMessage *msg)
{
    //receiving went wrong -> receiver did not answer
    if (msg == timeoutEvent) {
        int BTCR_max_repetitions = 2; //TODO
        if(packet_retry_number >= BTCR_max_repetitions)
        {
            EV << "node: " << this->getIndex() << "  Error: did not get reply -> abort" << endl;
            transmissions_failed++;
            sleep();
        }
        else //did not receive reply -> try again
        {
            packet_retry_number++;
            EV << "node: " << this->getIndex() << "  Error: did not get reply -> retry" << endl;
            //restart new packet after short delay
            scheduleAt(simTime()+uniform(0.01,0.1), endTxEvent);
            state = IDLE;
        }
    }

    else {
        if(state == IDLE)
        {
            packetNumber += 1;

            char packet_name_[64] = {"state==IDLE"};
            sendPacket(packet_name_, 0, 0);

            scheduleAt(simTime() + data_duration, endTxEvent);
            state = TRANSMIT_FINISHED;
        }
        else if(state == TRANSMIT_FINISHED)
        {
            //wait for receiver reply (ACK or NAK)
            scheduleAt(simTime() + timeout_duration, timeoutEvent);
            state = WAIT_FOR_REPLY;
        }
        else if(state == WAIT_TO_GET_SELECTED)
        {
            //do nothing
            EV << "node: " << this->getIndex() << "  Error: did not get reply -> retry" << endl;
        }
    }
}

void Host::self_message_CSMA(cMessage *msg)
{
    //receiving went wrong -> receiver did not answer
    if (msg == timeoutEvent) {
        EV << "node: " << this->getIndex() << "  Error: did not get reply -> retry" << endl;
        CSMA_retry_number++;
        CSMA_retry(0);
    }

    else {
        if(state == IDLE)
        {
            if(CSMA_cs_done == false && cs_enabled)
            {
                //check if channel is busy (single point check)
                channel_was_busy_during_sensing = carrier_sense();
                if(channel_was_busy_during_sensing == true)
                {
                    //skip packet
                    EV << "node: " << this->getIndex() << "  channel busy, skip packet" << endl;
                    packetNumber++;
                    CSMA_backoff_number++;
                    CSMA_retry(0);
                }
                else    //channel was not busy at start of cs -> send packet
                {
                    EV << "node: " << this->getIndex() << "  CS done: channel was not busy -> send packet" << endl;
                    CSMA_cs_done = true;
                    state = IDLE;
                    scheduleAt(simTime()+carrier_sense_duration+RX_TX_switching_time, endTxEvent);
                }
            }
            else //CS is done or deactivated -> send data
            {
                packetNumber++;
                char packet_name_[64] = {"state==IDLE"};
                sendPacket(packet_name_, 0, 0);

                scheduleAt(simTime() + data_duration, endTxEvent);
                state = TRANSMIT_FINISHED;
            }
        }
        else if(state == TRANSMIT_FINISHED)
        {
            //wait for receiver reply (ACK or NAK)
            scheduleAt(simTime() + timeout_duration, timeoutEvent);
            state = WAIT_FOR_REPLY;
        }
    }
}

double Host::CSMA_get_random_backoff()
{
    double backoff;

    int BE_temp;

    if(packetNumber < CSMA_min_BE)
        BE_temp = CSMA_min_BE;
    else if(packetNumber > CSMA_max_BE)
        BE_temp = CSMA_max_BE;
    else
        BE_temp = packetNumber;

    int pow_result = pow(2, BE_temp-1);
    int CSMA_contention_window = pow_result * CSMA_wc_min;
    if (CSMA_contention_window > CSMA_wc_max)   //limit contention window to CSMA_wc_max
        CSMA_contention_window = CSMA_wc_max;
    int rand = (int)uniform(0, CSMA_contention_window); //integer conversion to round down, rand number between [0,WC-1]
    if (rand > CSMA_contention_window)  rand = CSMA_contention_window;
    backoff = rand * CSMA_backoff_time;

    return backoff;
}

//schedule new packet after random backoff + delay
void Host::CSMA_retry(double delay_)
{
    double backoff;
    channel_was_busy_during_sensing = false;
    CSMA_cs_done = false;

    EV << "node: " << this->getIndex() << "  CSMA_retry() called" << endl;

    if(CSMA_retry_number >= CSMA_max_retransmissions)
    {
        EV << "node: " << this->getIndex() << "  max retransmission number reached (count: " << CSMA_retry_number << ")  SLEEP" << endl;
        transmissions_failed++;
        sleep();
        return;
    }
    else if(CSMA_backoff_number >= CSMA_max_backoffs)
    {
        EV << "node: " << this->getIndex() << "  max backoff number reached (count: " << CSMA_backoff_number << ")  SLEEP" << endl;
        transmissions_failed++;
        sleep();
        return;
    }

    backoff = CSMA_get_random_backoff();
    state = IDLE;

    EV << "node: " << this->getIndex() << "  backoff: " << backoff << "  schedule time: " << simTime()+backoff+delay_ << endl;

    scheduleAt(simTime()+backoff+delay_, endTxEvent);
}


//calculate this hosts slot number according to the id range and the total slot number
void Host::Proposed_calculate_host_slot()
{
    int host_id = this->getIndex();
    Proposed_host_slot = -1;
    for(int i=1; i<=Proposed_slot_number_current; i++)
    {
        if(Proposed_slot_list[i].higher >= host_id && Proposed_slot_list[i].lower <= host_id)
        {
            Proposed_host_slot = i;
            break;
        }
    }
}

//unused
void Host::Proposed_fill_slot_list()
{
    int diff;
    diff = (Proposed_higher_id-Proposed_lower_id);
    if(diff < Proposed_slot_number)
    {
        //EV << "host: id range too small" << endl;
        Proposed_slot_number_current = diff + 1;

        for(int i=1; i<=Proposed_slot_number_current; i++)
        {
            Proposed_slot_list[i].higher = Proposed_higher_id - (i-1);
            Proposed_slot_list[i].lower = Proposed_slot_list[i].higher;
            //EV << "debug: slot number: " << i << "  higher: " << Proposed_slot_list[i].higher << "  lower: " << Proposed_slot_list[i].lower << endl;
        }
        return;
    }
    else
        Proposed_slot_number_current = Proposed_slot_number;

    //calculate id ranges
    diff = (Proposed_higher_id-Proposed_lower_id) / Proposed_slot_number_current;
    for(int i=1; i<=Proposed_slot_number_current; i++)
    {
        Proposed_slot_list[i].higher = Proposed_higher_id - (i-1) * diff;
        Proposed_slot_list[i].lower = Proposed_slot_list[i].higher - diff+1;
    }
    if(Proposed_lower_id == 0)
        Proposed_slot_list[Proposed_slot_number_current].lower = 0;
}


void Host::sendPacket(char *name, double delay_, double duration_)
{
    char buf[64];
    char buf2[64];
    double duration_temp;

    EV << "node: "<< this->getIndex() << " sendPacket() called" << endl;

    if(duration_ == 0)
        duration_temp = data_duration;
    else
        duration_temp = duration_;

    // update network graphics
    if (ev.isGUI())
    {
        for(int i = 0; i<(int)sizeof(buf); i++)
            buf[i] = 0;
        sprintf(buf,"pk-id%d #%d r%d", getIndex(), packetNumber, packet_retry_number);

        sprintf(buf2, "TRANSMIT %i / %i", 1, packetNumberMax);
        getDisplayString().setTagArg("t",0,buf2);
    }

    //calculate delays
    delay_time_buf = simTime() - delay_time_stamp; //total time since wakeUp
    delay_time_buf += data_duration;
    message_delay = delay_time_buf.dbl();

    //calculate receive and transmit times
    switch(mode)
    {
    case 1://proposed
        tx_time = packetNumber * data_duration;
        rx_time = message_delay - tx_time; //we count RX_TX_switching_time, delay_after_ack, etc. as rx_time
        break;
    case 2://Strawman
        tx_time += data_duration;
        rx_time = message_delay - tx_time;
        break;
    case 3://BTCR
        tx_time = packetNumber * data_duration;
        rx_time = message_delay - tx_time;
        break;
    case 4://CSMA
        tx_time = packetNumber * data_duration;
        rx_time = packetNumber * (carrier_sense_duration + RX_TX_switching_time + delay_after_ack + ACK_duration);
        break;
    default:
        rx_time = -1;
        tx_time = -1;
        break;
    }

    //EV << "node: "<< this->getIndex() << " sendPacket() called delay: " << message_delay << "  tx_time: " << delay_time_buf << "  rx_time: " << rx_time << endl;

    cPacket *pk = new cPacket(name);
    pk->addPar("id");
    pk->par("id").setLongValue(this->getIndex());
    pk->addPar("repetition");
    pk->par("repetition").setLongValue(packetNumber);
    pk->addPar("delay");
    pk->par("delay").setDoubleValue(message_delay);
    pk->addPar("tx_time");
    pk->par("tx_time").setDoubleValue(tx_time);
    pk->addPar("rx_time");
    pk->par("rx_time").setDoubleValue(rx_time); //time for last ACK is added at receiver side
    pk->setBitLength(pkLenBits->longValue());


    if(delay_ == 0)
        sendDirect(pk, radioDelay, duration_temp, server->gate("in"));
    else
        sendDirect(pk, delay_, duration_temp, server->gate("in"));
        //delayed sending
        //sendDelayed(cMessage *msg, double delay, cGate *gate);
}

void Host::StrawmanSendContentionPacket(double length)
{
    EV << "node: " << this->getIndex() << "  send contention packet with length: " << length << endl;

    tx_time += length;

    cPacket *pk = new cPacket("CP");
    pk->addPar("Strawman_cp_packet");
    pk->addPar("id");
    pk->par("id").setLongValue(this->getIndex());
    pk->addPar("cp_length");
    pk->par("cp_length").setDoubleValue(length);
    pk->setBitLength(pkLenBits->longValue()); //not sure if bit length is required

    sendDirect(pk, 0, Strawman_contention_start_delay, server->gate("in"));
    //sendDirect(pk, Strawman_contention_start_delay, length, server->gate("in")); //produces errors since omnet++ messes up schedule order
    //sendDelayed(pk, length, server->gate("in"));
}

bool Host::carrier_sense()
{
    bool buf_;
    Server *server;
    for (SubmoduleIterator iter(getParentModule()); !iter.end(); iter++)
    {
        if (iter()->isName("server")) // if iter() is in the same vector as this module
        {
            server = check_and_cast<Server *>(iter());
            buf_ = server->channel_busy2();
            carrier_sense_done = true;
            break;
        }
    }
    return buf_;
}

double Host::max(double a, double b)
{
    if(a>b)
        return a;
    else
        return b;
}

void Host::reset_variables()
{
    //reset variables
    packetNumber = 0;
    packet_retry_number = 0;
    carrier_sense_done = false;
    channel_was_busy_during_sensing = false;
    message_delay = 0;
    cs_active = false;
    host_id_carrier_sense[getIndex()] = false;
    channel_is_busy = false;
    tx_time = 0;
    rx_time = 0;
    Proposed_TDMA_active = false;
    CSMA_cs_done = false;
    CSMA_backoff_number = 0;
    CSMA_retry_number = 0;
    desync = false;
    //ClockDriftPlotCurrentStepNumber = 0;
}

void Host::next_plot_step(int iteration_number)
{
    Enter_Method("next_plot_step()");

    ClockDriftPlotCurrentStepNumber++;
}


//used for CSMA (mode 5) csma_mode_2 only (for pausing back-off time)
void Host::update_channel_state(bool channel_busy)
{
    Enter_Method("update_channel_state()");

    /*if(channel_busy == old_channel_state)  //cancel, if called multiple times with same channel state
    {
        EV << "node: " << this->getIndex() << "  called with same channel state -> skip update_channel_state    channel_busy= " << channel_busy << endl;
        return;
    }
    else
    {
        EV << "node: "<< this->getIndex() << " called update_channel_state() with channel_busy= " << channel_busy << endl;
    }
    channel_is_busy = channel_busy;

    if(cs_active == true && carrier_sense_done == false && (mode == 5 && CSMA_mode == 2))
    {
        if(channel_busy == true) //channel busy during carrier sensing, pause backoff time
        {
            remaining_backoff_time = CSMA_backoff_finished_time - simTime();
            EV << "node: " << this->getIndex() << " pause carrier sensing -> cancelEvent(endCarrierSenseEvent)" << endl;
            cancelEvent(endCarrierSenseEvent);
        }
        else                        //channel is free again, resume backoff time
        {
            CSMA_backoff_finished_time = simTime() + remaining_backoff_time;
            EV << "node: " << this->getIndex() << " resume carrier sensing. Estimated duration: " << remaining_backoff_time
                            << " finish time: " << CSMA_backoff_finished_time << endl;
            scheduleAt(CSMA_backoff_finished_time, endCarrierSenseEvent);
        }
    }

    old_channel_state = channel_busy;*/
}

//calculates new double value with drift. Drift value must have already been calculated when calling this function
double Host::Clock_drift_calculate_period(double period)
{
    double periodTime_buf;
    if(ClockDriftEnabled == false)
        return period;

    if(Clock_drift_current_value < 0)
        periodTime_buf = period * (1-Clock_drift_current_value);
    else
        periodTime_buf = period * (1 + Clock_drift_current_value);

    return periodTime_buf;
}

void Host::Clock_drift_generate_new_random_drift()
{
    //clock drift
    if(ClockDriftEnabled == true)
    {
        if(ClockDriftPlotMode == true)
            ClockDriftPlotDriftRange_temp = (ClockDriftRangePercent / ClockDriftPlotStepNumber) * ClockDriftPlotCurrentStepNumber;
        else
            ClockDriftPlotDriftRange_temp = ClockDriftRangePercent;

        //calculate current drift
        Clock_drift_current_value = uniform(-ClockDriftPlotDriftRange_temp, +ClockDriftPlotDriftRange_temp);
    }
    else
    {
        //periodTime = periodTime_original;
        Clock_drift_current_value = 0;
    }
}

long int Host::collect_and_clear_transmission_successful()
{
    Enter_Method("collect_and_clear_packet_successful()");
    //EV << "node: " << this->getIndex() <<  "  collect_and_clear_transmission_successful() called; transmissions_successful: " << transmissions_successful << endl;

    long int buf = transmissions_successful;
    transmissions_successful = 0;
    return buf;
}
long int Host::collect_and_clear_transmission_failed()
{
    Enter_Method("collect_and_clear_packet_failed()");
    //EV << "node: " << this->getIndex() <<  "  collect_and_clear_transmission_failed() called; transmissions_failed: " << transmissions_failed << endl;

    long int buf = transmissions_failed;
    transmissions_failed = 0;
    return buf;
}


}; //namespace
