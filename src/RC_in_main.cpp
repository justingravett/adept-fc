//
// Use of this file is governed by the MIT License - see adept_fc/LICENSE_MIT
//
// Copyright (c) 2019 Timothy Bretl, Aaron Perry, and Phillip Ansell
//

//Acquires RC input channel values and publishes them

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <zcm/zcm-cpp.hpp>
#include <Navio2/RCInput_Navio2.h>
#include <Common/Util.h>
#include <memory>
#include <chrono>
//message types:
#include "rc_t.hpp"
#include "status_t.hpp"
#include "adc_data_t.hpp"

#define READ_FAILED -1

using std::string;

class Handler
{
    public:
        ~Handler() = default;

        status_t stat;
        adc_data_t adc;

        Handler()
        {
            memset(&stat,0,sizeof(stat));
            memset(&adc,0,sizeof(adc));
        }

        void read_stat(const zcm::ReceiveBuffer* rbuf,const string& chan,const status_t *msg)
        {
            stat = *msg;
        }

        void read_adc(const zcm::ReceiveBuffer* rbuf,const string& chan,const adc_data_t *msg)
        {
            adc = *msg;
        }
};

double get_gps_time(Handler* adchandle)
{
    double adc_gps = adchandle->adc.time_gps;
    int64_t adc_time = adchandle->adc.time_rpi;
    int64_t rpi_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    return  adc_gps + (rpi_time - adc_time)/1000000.0;
}

std::unique_ptr <RCInput> get_rcin()
{
    auto ptr = std::unique_ptr <RCInput>{ new RCInput_Navio2() };
    return ptr;
}

int main()
{
    //load configuration
    int rc_fail_servo = 1500;
    int rc_fail_esc = 1000;
    bool err_on = true;
    string dump;
    std::ifstream config_stream;

    config_stream.open("/home/pi/adept-fc/config_files/rc_in.config");
    config_stream >> dump >> rc_fail_servo;
    config_stream >> dump >> rc_fail_esc;
    config_stream >> dump >> dump;
    err_on = !dump.compare("true");
    config_stream.close();

    //initialize ZCM
    zcm::ZCM zcm {"ipc"};

    status_t module_stat;
    memset(&module_stat,0,sizeof(module_stat));
    module_stat.module_status = 1;//module running

    //subscribe to incoming channels:
    Handler handlerObject;
    zcm.subscribe("STATUS",&Handler::read_stat,&handlerObject);
    zcm.subscribe("ADC_DATA",&Handler::read_adc,&handlerObject);

    //initialize message objects
    rc_t rc_in;
    memset(&rc_in,0,sizeof(rc_in));

    //initialize RC input
    if (check_apm())
    {
        return 1;
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
        " RC_IN: failed to initialize rc input." << std::endl;
    }
    auto rcin = get_rcin();
    rcin->initialize();

    zcm.start();

    std::cout<< "rc_in started" << std::endl;

    while (!handlerObject.stat.should_exit)
    {
        zcm.publish("STATUS0",&module_stat);
        //read in each of the 8 channels:
        for (int i = 0; i<8; i++)
        {

            rc_in.rc_chan[i]=rcin->read(i);

            if ((rc_in.rc_chan[i] == READ_FAILED) || (rc_in.rc_chan[i]<500))
            {
                //failsafe logic
                if (i<3)
                {
                    rc_in.rc_chan[i] = rc_fail_servo;
                }
                else if (i==3)
                {
                    rc_in.rc_chan[i] = rc_fail_esc;
                }
                else if (i==4)//consider removing hard code on this one
                {
                    rc_in.rc_chan[i] = 1000; //force manual flight mode to register
                }
                //log an error message
                if (err_on)
                {
                    std::cout <<  std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
                    " RC_IN: error: RC read fail" << std::endl;
                }
            }

        }
        //timestamp the data
        rc_in.time_gps = get_gps_time(&handlerObject);

        //publish the RC values
        zcm.publish("RC_IN", &rc_in);
        usleep(10000);
    }

    module_stat.module_status = 0;
    zcm.publish("STATUS0",&module_stat);

    std::cout << "rc_in module exiting..." << std::endl;

    zcm.stop();

    return 0;
}
