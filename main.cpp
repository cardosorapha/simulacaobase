//author  Renato Sousa, 2018
#include <QtNetwork>
#include <stdio.h>
#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"

#include "net/pb/command.pb.h"
#include "net/pb/common.pb.h"
#include "net/pb/packet.pb.h"
#include "net/pb/replacement.pb.h"

#include "strategy.h"

#include <iostream>
#include <sstream>

using namespace std;
/*
void printRobotInfo(const fira_message::Robot & robot) {

    printf("ID=%3d \n",robot.robot_id());

    printf(" POS=<%9.2f,%9.2f> \n",robot.x(),robot.y());
    printf(" VEL=<%9.2f,%9.2f> \n",robot.vx(),robot.vy());

    printf("ANGLE=%6.3f \n",robot.orientation());
    printf("ANGLE VEL=%6.3f \n",robot.vorientation());
}
*/
int main(int argc, char *argv[]){
    //(void)argc;
    //(void)argv;
    /*
    printf(argv[1]);
    printf("\n");
    printf(argv[2]);
    printf("\n");
    printf(argv[3]);
    printf("\n");
    printf(argv[4]);
    printf("\n");
    */
    string IP;
    string vision;
    string command;
    string campo;

    if(argc == 5){ // se existir entrada via terminal
        IP = argv[1];
        vision = argv[2];
        command = argv[3];
        campo = argv[4];
    }else{         //se não existir executa os valores abaixo
        IP = "224.0.0.1";
        vision = "10020";
        command = "20011";
        campo = "amarelo";
    }

    stringstream aux(vision);
    int visao = 0;
    aux >> visao;

    stringstream aux2(command);
    int comando = 0;
    aux2 >> comando;

    bool my_robots_are_yellow;
    bool ambos;
    if (campo == "azul"){
        my_robots_are_yellow = false;
        ambos = false;
    }else if (campo == "amarelo"){
            my_robots_are_yellow = true;
            ambos = false;
    }else if (campo == "ambos"){
            my_robots_are_yellow = true;
            ambos = true;
    }else{
        printf("ERRO ESCOLHA DE campo");
        exit(EXIT_FAILURE);
    }

    // the ip address need to be in the range 224.0.0.0 through 239.255.255.255
    RoboCupSSLClient visionClient(IP, visao);
    visionClient.open(false);

    GrSim_Client commandClient("127.0.0.1", comando);

    fira_message::sim_to_ref::Environment packet;

    //inicialização da classe estratégia

    //inicializa duas estrategias uma para cada time em caso de ambos os times serem utilizados
    Strategy estrategia_amarela(my_robots_are_yellow);
    Strategy estrategia_azul(!my_robots_are_yellow);

    //inicializa estrategia azul ou amarela
    Strategy estrategia(my_robots_are_yellow);

    while(true) {
        if (visionClient.receive(packet)) {
            //printf("-----Received Wrapper Packet---------------------------------------------\n");
            //see if the packet contains a robot detection frame:
            if ((packet.has_frame())&&(packet.has_field())) {
                fira_message::Frame detection = packet.frame();

                int robots_blue_n =  detection.robots_blue_size();
                int robots_yellow_n =  detection.robots_yellow_size();
                //Ball info:
                fira_message::Ball ball = detection.ball();

                if (ambos){
                    estrategia_azul.predict_ball(ball);
                    estrategia_amarela.predict_ball(ball);
                }else{
                    estrategia.predict_ball(ball);
                }

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
                fira_message::Robot b0 = detection.robots_blue(0);
                fira_message::Robot b1 = detection.robots_blue(1);
                fira_message::Robot b2 = detection.robots_blue(2);
                //Yellow
                fira_message::Robot y0 = detection.robots_yellow(0);
                fira_message::Robot y1 = detection.robots_yellow(1);
                fira_message::Robot y2 = detection.robots_yellow(2);


                if(ambos){
                    estrategia_amarela.atualiza_pos(b0,b1,b2,y0,y1,y2);
                    estrategia_azul.atualiza_pos(b0,b1,b2,y0,y1,y2);

                    estrategia_amarela.strategy_yellow(y0,y1,y2,b0,b1,b2,ball,field);
                    estrategia_azul.strategy_blue(b0,b1,b2,y0,y1,y2,ball,field);

                    //Enviando velocidades para os robos amarelos
                    for(int i = 0;i < estrategia_amarela.qtdRobos;i++)
                        commandClient.sendCommand(estrategia_amarela.vRL[i][1],estrategia_amarela.vRL[i][0],my_robots_are_yellow,i);

                    //Enviando velocidades para os robos azuis
                    for(int i = 0;i < estrategia_azul.qtdRobos;i++)
                        commandClient.sendCommand(estrategia_azul.vRL[i][1],estrategia_azul.vRL[i][0],!my_robots_are_yellow,i);

                }else{
                    estrategia.atualiza_pos(b0,b1,b2,y0,y1,y2);

                    if(my_robots_are_yellow){
                        estrategia.strategy_yellow(y0,y1,y2,b0,b1,b2,ball,field);
                    }else{
                        estrategia.strategy_blue(b0,b1,b2,y0,y1,y2,ball,field);
                    }
                    //Enviando velocidades para os robos
                    for(int i = 0;i < estrategia.qtdRobos;i++)
                        commandClient.sendCommand(estrategia.vRL[i][1],estrategia.vRL[i][0],my_robots_are_yellow,i);

                }


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
