//
// Use of this file is governed by the MIT License - see adept_fc/LICENSE_MIT
//
// Copyright (c) 2019 Timothy Bretl, Aaron Perry, and Phillip Ansell
//

//Reads in actuator commands and sends PWM commands out

#include <zcm/zcm-cpp.hpp>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "Navio2/PWM.h"
#include "Navio2/RCOutput_Navio2.h"
#include "Common/Util.h"
#include <memory>
#include <chrono>
//message types:
#include "pwm_t.hpp"
#include "rc_t.hpp"
#include "actuators_t.hpp"
#include "status_t.hpp"
#include "adc_data_t.hpp"
#include "vnins_data_t.hpp"

using std::string;

class Handler
{
public:
~Handler() = default;

rc_t rc_in;
status_t stat;
adc_data_t adc;
vnins_data_t vnins;

double acts[11]={0};
int mode_emergency = 0;
int rc_emergency = 0;
int64_t last_rc_time = 0;
int64_t last_act_time = 0;
int64_t last_vnins_time = 0;
int64_t last_adc_time = 0;

Handler()
{
        memset(&rc_in,0,sizeof(rc_in));
        memset(&stat,0,sizeof(stat));
        memset(&adc,0,sizeof(adc));
        memset(&vnins,0,sizeof(vnins));
}

void read_rc(const zcm::ReceiveBuffer* rbuf,const string& chan,const rc_t *msg)
{
        rc_in = *msg;
        last_rc_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void read_adc(const zcm::ReceiveBuffer* rbuf,const string& chan,const adc_data_t *msg)
{
        adc = *msg;
        last_adc_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void read_vn200(const zcm::ReceiveBuffer* rbuf,const string& chan,const vnins_data_t *msg)
{
        vnins = *msg;
        last_vnins_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void read_stat(const zcm::ReceiveBuffer* rbuf,const string& chan,const status_t *msg)
{
        stat = *msg;
}

void read_acts(const zcm::ReceiveBuffer* rbuf,const string& chan,const actuators_t *msg)
{
        last_act_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        acts[0] = msg->da;
        acts[1] = msg->de;
        acts[2] = msg->dr;
        for (int i=3; i<11; i++)
        {
                acts[i] = msg->dt[i-3];
        }
}

};

std::unique_ptr <RCOutput> get_rcout()
{
        auto ptr = std::unique_ptr <RCOutput>{ new RCOutput_Navio2() };
        return ptr;
}

int output_scaling(const int& in_val, const double& s_min, const double& s_max, const int& in_min, const int& in_max)
{
        return (s_min + (s_max-s_min)/(in_max-in_min)*(in_val-in_min));
}

//lookup table function (multisine overlay)
void get_ctrl(int T_Lim, double current_time, double *time_vect, double *ele, double *ail, double *rud, double thr[][8], double ctr_out[11])
{

        if(current_time<T_Lim)
        { // use the multisine manuver as a delta to the manual controls
                bool foundtime = false;
                int i = 0;
                do //loop through the time vector until we have found the matching discrete value
                {

                        i++;

                        if (current_time<time_vect[i]) //we have found the correct time bin
                        {
                                foundtime = true;
                                //set the outputs:
                                ctr_out[0]= ail[i-1];
                                ctr_out[1] = ele[i-1];
                                ctr_out[2] = rud[i-1];
                                ctr_out[3] = thr[i-1][0];
                                ctr_out[4] = thr[i-1][1];
                                ctr_out[5] = thr[i-1][2];
                                ctr_out[6] = thr[i-1][3];
                                ctr_out[7] = thr[i-1][4];
                                ctr_out[8] = thr[i-1][5];
                                ctr_out[9] = thr[i-1][6];
                                ctr_out[10] = thr[i-1][7];
                        }
                } while(!foundtime);
        }
        else //if the maneuver is done, apply a delta of 0.0 to the manual controls
        {
                ctr_out[0]= 0.0;
                ctr_out[1] = 0.0;
                ctr_out[2] = 0.0;
                ctr_out[3] = 0.0;
                ctr_out[4] = 0.0;
                ctr_out[5] = 0.0;
                ctr_out[6] = 0.0;
                ctr_out[7] = 0.0;
                ctr_out[8] = 0.0;
                ctr_out[9] = 0.0;
                ctr_out[10] = 0.0;
        }
        return;
}

double get_gps_time(Handler* adchandle)
{
        double adc_gps = adchandle->adc.time_gps;
        int64_t adc_time = adchandle->adc.time_rpi;
        int64_t rpi_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        return adc_gps + (rpi_time - adc_time)/1000000.0;
}


int main(int argc, char *argv[])
{


        //load configuration variables
        string dump;
        int mapping[11] = {0,1,2,3,3,3,3,3,3,3,3};
        int num_outputs = 11;
        int pwm_freq = 50;
        int disarm_pwm_servo = 1500;
        int disarm_pwm_esc   = 1100;
        int servo_min = 1100;
        int servo_max = 1900;
        int rc_min = 1000;
        int rc_max = 1995;
        float surface_max[11] = {0};
        float surface_min[11] = {0};
        int mode_chan = 4;
        int mode_cutoff = 1500;
        int maneuver_chan = 5;
        int gain_chan = 6;
        double ele_zero_cmd = 1516.0;
        double ail_zero_cmd = 1494.0;
        double rud_zero_cmd = 1546.0;
        double thr_zero_cmd = 1085.0;
        double null_zone = 0.005;
        int pilot_cmd = 0;
        int ap_cmd = 0;
        int use_cmd = 0;
        int itr_ail_zero = 0;
        int itr_ele_zero = 0;
        int itr_rud_zero = 0;
        int wait_time = 3;
        int itr = 1;
        std::ifstream config_stream;

        // This function determines the scaling for PWM conversion
        for(int i=3; i<11; i++)
        {
                surface_min[i] = (float)rc_min;
                surface_max[i] = (float)rc_max;
        }


        config_stream.open("/home/pi/adept-fc/config_files/pwm_out.config");
        config_stream >> dump >> pwm_freq;
        config_stream >> dump >> disarm_pwm_servo;
        config_stream >> dump >> disarm_pwm_esc;
        config_stream >> dump >> servo_min;
        config_stream >> dump >> servo_max;
        config_stream >> dump >> surface_min[0] >> surface_min[1] >> surface_min[2];
        config_stream >> dump >> surface_max[0] >> surface_max[1] >> surface_max[2];
        config_stream >> dump >> rc_min;
        config_stream >> dump >> rc_max;
        config_stream >> dump >> mode_chan;
        config_stream >> dump >> mode_cutoff;
        config_stream >> dump >> maneuver_chan;
        config_stream >> dump >> gain_chan;
        config_stream.close();
        mode_chan--;
        maneuver_chan--;
        gain_chan--;

        //load flight test maneuver:
        float k[3][11] = {0};
        int N = 501;
        int Time_Limit = 10;

        std::ifstream in_stream,gain_stream;
        gain_stream.open("/home/pi/adept-fc/config_files/gains_multisine.dat");
        gain_stream >> dump >> k[0][0] >> dump >> k[1][0] >> dump >> k[2][0];//Read aileron gains
        gain_stream >> dump >> k[0][1] >> dump >> k[1][1] >> dump >> k[2][1];//Read elevator gains
        gain_stream >> dump >> k[0][2] >> dump >> k[1][2] >> dump >> k[2][2];//Read rudder gains
        gain_stream >> dump >> k[0][3] >> dump >> k[1][3] >> dump >> k[2][3];//Read throttle gains
        gain_stream.close();
        gain_stream.clear();

        for (int i=4; i<11; i++)
        {
                k[0][i] = k[0][3];
                k[1][i] = k[1][3];
                k[2][i] = k[2][3];
        }

        in_stream.open("/home/pi/adept-fc/config_files/multisine.dat");
        in_stream >> N;
        in_stream >> Time_Limit;
        double elevator[N],aileron[N],rudder[N],time_vect[N], Delta_Throttle[N][8];

        for(int i=0; i<N; i++)
        {
                in_stream >> time_vect[i] >> elevator[i] >> rudder[i] >> aileron[i]
                >> Delta_Throttle[i][0] >> Delta_Throttle[i][1] >> Delta_Throttle[i][2]
                >> Delta_Throttle[i][3] >> Delta_Throttle[i][4] >> Delta_Throttle[i][5]
                >> Delta_Throttle[i][6] >> Delta_Throttle[i][7];
        }
        in_stream.close();
        in_stream.clear();

        //initialize timing variables
        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> _T; //Local time since mode switch
        int prev_status = 0;
        int man_run = 0;
        double multisine_output[11];
        int gainpick = 0;

        //initialize zcm
        zcm::ZCM zcm {"ipc"};

        //structures to publish
        pwm_t pwm_comm;
        memset(&pwm_comm, 0, sizeof(pwm_comm));


        //subscribe to incoming channels:
        Handler handlerObject,sens_handler;
        zcm.subscribe("RC_IN",&Handler::read_rc,&handlerObject);
        zcm.subscribe("STATUS",&Handler::read_stat,&handlerObject);
        zcm.subscribe("ACTUATORS",&Handler::read_acts,&handlerObject);
        zcm.subscribe("ADC_DATA",&Handler::read_adc,&sens_handler);
        zcm.subscribe("VNINS_DATA",&Handler::read_vn200,&sens_handler);

        //for publishing stat of this module
        status_t module_stat;
        memset(&module_stat,0,sizeof(module_stat));
        module_stat.module_status = 1;//module running

        //initialize PWM outputs
        //************************************************************
        auto pwm = get_rcout();

        if (check_apm())
        {
                return 1;
        }

        if (getuid())
        {
                std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
                        " Not root. Please launch with Sudo." << std::endl;
        }

        for (int i=0; i<num_outputs; i++)
        {

                if( !(pwm->initialize(i)) )
                {
                        return 1;
                }

                pwm->set_frequency(i, pwm_freq);

                if ( !(pwm->enable(i)) )
                {
                        return 1;
                }

                usleep(50000); //without this, initialization of multiple channels fails
        }

        //set disarm pwm values
        for (int i = 0; i<num_outputs; i++)
        {
                if (i<=2)
                {
                        pwm_comm.pwm_out[i] = disarm_pwm_servo;
                }
                else
                {
                        pwm_comm.pwm_out[i] = disarm_pwm_esc;
                }
        }
        //************************************************************

        zcm.start();

        std::cout<< "pwm_out started" << std::endl;

        //initialize emergency message loss detection
        bool message_thrown[2] = {false,false};
        handlerObject.last_act_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        handlerObject.last_rc_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        sens_handler.last_adc_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        sens_handler.last_vnins_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

        //check for an emergency startup
        std::ifstream f("/home/pi/adept-fc/emergency_startup");
        if (f.good())
        {
                handlerObject.stat.armed = 1;
                std::cout << "Armed on emergency startup" << std::endl;
        }


        while (!handlerObject.stat.should_exit)
        {
                zcm.publish("STATUS6",&module_stat);
                //maneuver lookup
                man_run = handlerObject.rc_in.rc_chan[maneuver_chan];
                if (man_run >= 1500)
                {
                        if(prev_status < 1500)
                        {
                                start = std::chrono::high_resolution_clock::now();
                        }
                        //Compute time since mode switch:
                        _T = std::chrono::duration_cast<std::chrono::duration<double> >(std::chrono::high_resolution_clock::now() - start);

                        get_ctrl(Time_Limit, _T.count(),time_vect,elevator,aileron,rudder,Delta_Throttle,multisine_output);
                }
                else
                {
                        //normal operation
                        memset(&multisine_output, 0, sizeof(multisine_output));
                }

                prev_status = man_run;

                //get maneuver gain:
                if (handlerObject.rc_in.rc_chan[gain_chan] > 1750)
                {
                        gainpick = 2;
                }
                else if (handlerObject.rc_in.rc_chan[gain_chan] < 1250)
                {
                        gainpick = 0;
                }
                else
                {
                        gainpick = 1;
                }

                //handle emergency logic:
                int64_t current_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
                //if we lose actuator, adc, or vnins data, autopilot cannot be in co
                if ((current_time-handlerObject.last_act_time < 500000) && (current_time-sens_handler.last_adc_time < 500000) && (current_time-sens_handler.last_vnins_time < 500000))
                {
                        handlerObject.mode_emergency = 0;
                        message_thrown[1] = false;
                }
                else
                {
                        handlerObject.mode_emergency = 1;
                        if (message_thrown[1] == false)
                        {
                                std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
                                        " WARNING: Mode emergency detected. Switching to manual flight mode." << std::endl;
                                message_thrown[1] = true;
                        }
                }
                if (current_time-handlerObject.last_rc_time < 500000)
                {
                        handlerObject.rc_emergency = 0;
                        message_thrown[0] = false;
                }
                else
                {
                        handlerObject.rc_emergency = 1;
                        if (message_thrown[0] == false)
                        {
                                std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
                                        " WARNING: RC emergency detected. Switching to disarm." << std::endl;
                                message_thrown[0] = true;
                        }
                }

                //pwm output logic
                if (handlerObject.stat.armed && handlerObject.rc_emergency == 0)
                {
                        if (handlerObject.rc_in.rc_chan[mode_chan]<mode_cutoff || handlerObject.mode_emergency == 1) //manual flight mode:
                        {
                                for (int i=0; i<num_outputs; i++)
                                {
                                        pwm_comm.pwm_out[i] = k[gainpick][i]*multisine_output[i] + output_scaling(handlerObject.rc_in.rc_chan[mapping[i]],servo_min,servo_max,rc_min,rc_max);
                                        pwm->set_duty_cycle(i, pwm_comm.pwm_out[i]);
                                }
                                if (itr % 500 == 0)
                                {
                                        itr = 1;
                                }
                        }
                        else if (handlerObject.rc_in.rc_chan[mode_chan]>=mode_cutoff && handlerObject.mode_emergency == 0) //auto flight mode:
                        {
                                for (int i=0; i<num_outputs; i++) // Ignore autopilot commands if pilot commands are present
                                {
                                        // Gather Pilot and AP cmds
                                        pilot_cmd = output_scaling(handlerObject.rc_in.rc_chan[mapping[i]],servo_min,servo_max,rc_min,rc_max);
                                        ap_cmd = (int)(handlerObject.acts[i]);

                                        if (i == 0) // Aileron command
                                        {
                                                if ((pilot_cmd > (int)((1.0+null_zone/2.0)*ail_zero_cmd)) || (pilot_cmd < (int)((1.0-null_zone/2.0)*ail_zero_cmd)))
                                                {
                                                        use_cmd = pilot_cmd;
                                                        itr_ail_zero = 0;
                                                }
                                                else
                                                {
                                                        if (itr_ail_zero < 100*wait_time) // Wait 2 seconds after 0 input to engage autopilot
                                                        {
                                                                use_cmd = pilot_cmd;
                                                                itr_ail_zero++;
                                                        }
                                                        else
                                                        {
                                                                use_cmd = ap_cmd;
                                                        }
                                                }
                                        }
                                        else if (i == 1) // Elevator command
                                        {
                                                if ((pilot_cmd > (int)((1.0+null_zone/2.0)*ele_zero_cmd)) || (pilot_cmd < (int)((1.0-null_zone/2.0)*ele_zero_cmd)))
                                                {
                                                        use_cmd = pilot_cmd;
                                                        itr_ele_zero = 0;
                                                }
                                                else
                                                {
                                                        if (itr_ele_zero < 100*wait_time) // Wait wait_time seconds after 0 input to engage autopilot
                                                        {
                                                                use_cmd = pilot_cmd;
                                                                itr_ele_zero++;
                                                        }
                                                        else
                                                        {
                                                                use_cmd = ap_cmd;
                                                        }
                                                }
                                        }
                                        else if (i == 2) // Rudder command
                                        {
                                                if ((pilot_cmd > (int)((1.0+null_zone/2.0)*rud_zero_cmd)) || (pilot_cmd < (int)((1.0-null_zone/2.0)*rud_zero_cmd)))
                                                {
                                                        use_cmd = pilot_cmd;
                                                        itr_rud_zero = 0;
                                                }
                                                else
                                                {
                                                        if (itr_rud_zero < 100*wait_time) // Wait wait_time seconds after 0 input to engage autopilot
                                                        {
                                                                use_cmd = pilot_cmd;
                                                                itr_rud_zero++;
                                                        }
                                                        else
                                                        {
                                                                use_cmd = ap_cmd;
                                                        }
                                                }
                                        }
                                        else // Thrust command
                                        {
                                                use_cmd = (pilot_cmd > (int)((1.0+null_zone/2.0)*thr_zero_cmd)) || (pilot_cmd < (int)((1.0-null_zone/2.0)*thr_zero_cmd)) ? pilot_cmd : ap_cmd;
                                        }

                                        // Send cmd to PWM
                                        pwm_comm.pwm_out[i] = use_cmd;
                                        pwm->set_duty_cycle(i, pwm_comm.pwm_out[i]);
                                }
                                if (itr % 500 == 0)
                                {
                                        itr = 1;
                                }
                        }
                }
                else //disarmed
                {
                        for (int i = 0; i<num_outputs; i++)
                        {
                                if (i<=2)
                                {
                                        pwm_comm.pwm_out[i] = disarm_pwm_servo;
                                        pwm->set_duty_cycle(i, pwm_comm.pwm_out[i]);
                                }
                                else
                                {
                                        pwm_comm.pwm_out[i] = disarm_pwm_esc;
                                        pwm->set_duty_cycle(i, pwm_comm.pwm_out[i]);
                                }
                        }

                }
                //timestamp the data
                pwm_comm.time_gps = get_gps_time(&sens_handler);
                itr++;

                //publish pwm values for logging
                zcm.publish("PWM_OUT", &pwm_comm);
                usleep(10000);
        }

        module_stat.module_status = 0;
        zcm.publish("STATUS6",&module_stat);

        zcm.stop();
        std::cout << "pwm_out module exiting..." << std::endl;
        return 0;
}
