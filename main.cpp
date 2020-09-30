//author  GPRUFS 2020
#include <QtNetwork>
#include <stdio.h>
#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"

#include "pb/command.pb.h"
#include "pb/common.pb.h"
#include "pb/packet.pb.h"
#include "pb/replacement.pb.h"

#include "strategy.h"

#include <iostream>

using namespace std;

void printRobotInfo(const fira_message::Robot & robot) {

    printf("ID=%3d \n",robot.robot_id());

    printf(" POS=<%9.2f,%9.2f> \n",robot.x(),robot.y());
    printf(" VEL=<%9.2f,%9.2f> \n",robot.vx(),robot.vy());

    printf("ANGLE=%6.3f \n",robot.orientation());
    printf("ANGLE VEL=%6.3f \n",robot.vorientation());
}

int main(int argc, char *argv[]){
    (void)argc;
    (void)argv;
    RoboCupSSLClient client;
    client.open(false);
    fira_message::sim_to_ref::Environment packet;

    GrSim_Client grSim_client;

    while(true) {
        if (client.receive(packet)) {
            //printf("-----Received Wrapper Packet---------------------------------------------\n");
            //see if the packet contains a robot detection frame:
            if ((packet.has_frame())&&(packet.has_field())) {
                fira_message::Frame detection = packet.frame();

                int robots_blue_n =  detection.robots_blue_size();
                int robots_yellow_n =  detection.robots_yellow_size();
                //Ball info:
                fira_message::Ball ball = detection.ball();
                //printf("-Ball:  POS=<%9.2f,%9.2f> \n",ball.x(),ball.y());

                //printf("-[Geometry Data]-------\n");
                const fira_message::Field & field = packet.field();
                //printf("Field Dimensions:\n");
                //printf("  -field_length=%f (mm)\n",field.length());
                //printf("  -field_width=%f (mm)\n",field.width());
                //printf("  -goal_width=%f (mm)\n",field.goal_width());
                //printf("  -goal_depth=%f (mm)\n",field.goal_depth());


                //Robots info
                //Blue
               // vector<fira_message::Robot>* blue = NULL;
               // blue = new vector<fira_message::Robot>(3);
                //blue->push_back(detection.robots_blue(0));
                //blue->push_back(detection.robots_blue(1));
                //blue->push_back(detection.robots_blue(2));

                fira_message::Robot b0 = detection.robots_blue(0);
                fira_message::Robot b1 = detection.robots_blue(1);
                fira_message::Robot b2 = detection.robots_blue(2);

                //Yellow
                //vector<fira_message::Robot>* yellow = NULL;
               // yellow = new vector<fira_message::Robot>(3);
               // yellow->push_back(detection.robots_yellow(0));
               // yellow->push_back(detection.robots_yellow(1));
                //yellow->push_back(detection.robots_yellow(2));

                fira_message::Robot y0 = detection.robots_yellow(0);
                fira_message::Robot y1 = detection.robots_yellow(1);
                fira_message::Robot y2 = detection.robots_yellow(2);

                Strategy estrategia;
                estrategia.strategy_blue(b0,b1,b2,y0,y1,y2,ball,field);

                //Enviando velocidades
                for(int i = 0;i < estrategia.qtdRobos;i++)
                    grSim_client.sendCommand(estrategia.vRL[i][1],estrategia.vRL[i][0],i);

                //delete blue;
                //delete yellow;

                //Debug
                //printf("V:%f\n",sqrt(pow(b2.vx(),2)+pow(b2.vy(),2)));
                //printf("W:%f\n",b2.vorientation());
                //printf("Venviado:%f\n",estrategia.VW[2][0]);
                //printf("Wenviado:%f\n",estrategia.VW[2][1]);
                //printf("VR:%f\n",estrategia.vRL[2][0]);
                //printf("VL:%f\n",estrategia.vRL[2][1]);

            }
        }
    }

    return 0;
}
