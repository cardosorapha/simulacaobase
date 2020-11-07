//author  Renato Sousa, 2018
//modificações TIME LAMBE SUJO 2020
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

#include "net/vssclient.h"

#define UDP_ADDRESS "224.5.23.2"
#define REFEREE_PORT 10003
#define REPLACER_PORT 10004

QString getFoulNameById(VSSRef::Foul foul){
    switch(foul){
        case VSSRef::Foul::FREE_BALL:    return "FREE_BALL";
        case VSSRef::Foul::FREE_KICK:    return "FREE_KICK";
        case VSSRef::Foul::GOAL_KICK:    return "GOAL_KICK";
        case VSSRef::Foul::PENALTY_KICK: return "PENALTY_KICK";
        case VSSRef::Foul::KICKOFF:      return "KICKOFF";
        case VSSRef::Foul::STOP:         return "STOP";
        case VSSRef::Foul::GAME_ON:      return "GAME_ON";
        default:                         return "FOUL NOT IDENTIFIED";
    }
}

QString getTeamColorNameById(VSSRef::Color color){
    switch(color){
        case VSSRef::Color::NONE:    return "NONE";
        case VSSRef::Color::BLUE:    return "BLUE";
        case VSSRef::Color::YELLOW:  return "YELLOW";
        default:                     return "COLOR NOT IDENTIFIED";
    }
}

QString getQuadrantNameById(VSSRef::Quadrant quadrant){
    switch(quadrant){
        case VSSRef::Quadrant::NO_QUADRANT: return "NO QUADRANT";
        case VSSRef::Quadrant::QUADRANT_1:  return "QUADRANT 1";
        case VSSRef::Quadrant::QUADRANT_2:  return "QUADRANT 2";
        case VSSRef::Quadrant::QUADRANT_3:  return "QUADRANT 3";
        case VSSRef::Quadrant::QUADRANT_4:  return "QUADRANT 4";
        default:                            return "QUADRANT NOT IDENTIFIED";
    }
}

QString getHalfNameById(VSSRef::Half half){
    switch(half){
        case VSSRef::Half::NO_HALF: return "NO_HALF";
        case VSSRef::Half::FIRST_HALF: return "FIRST HALF";
        case VSSRef::Half::SECOND_HALF: return "SECOND HALF";
        default: "NO HALF DEFINED";
    }
}

using namespace std;

int main(int argc, char *argv[]){

    QUdpSocket *replacerSocket = new QUdpSocket();
    VSSClient *client = new VSSClient(REFEREE_PORT, UDP_ADDRESS);

    // Performing connection to send Replacer commands
    if(replacerSocket->isOpen())
        replacerSocket->close();

    replacerSocket->connectToHost(UDP_ADDRESS, REPLACER_PORT, QIODevice::WriteOnly, QAbstractSocket::IPv4Protocol);
    std::cout << "[Example] Connected to REPLACER socket in port " << REPLACER_PORT << " and address = " << UDP_ADDRESS << ".\n";

    // Performing connection to receive Referee foul commands
    if(client->open(true))
        std::cout << "[Example] Listening to referee system on port " << REFEREE_PORT << " and address = " << UDP_ADDRESS << ".\n";
    else{
        std::cout << "[Example] Cannot listen to referee system on port " << REFEREE_PORT << " and address = " << UDP_ADDRESS << ".\n";
        return 0;
    }

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
        //JOGO RODANDO
        if (visionClient.receive(packet)) {
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

                const fira_message::Field & field = packet.field();
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

            }
        }
        VSSRef::ref_to_team::VSSRef_Command command;

        //JUIZ ENVIOU COMANDO
        if(client->receive(command)){
            // If received command, let's debug it
            std::cout << "[Example] Succesfully received an command from ref: " << getFoulNameById(command.foul()).toStdString() << " for team " << getTeamColorNameById(command.teamcolor()).toStdString() << std::endl;
            std::cout << getQuadrantNameById(command.foulquadrant()).toStdString() << std::endl;

            // Showing timestamp
            std::cout << "Timestamp: " << command.timestamp() << std::endl;

            // Showing half
            std::cout << "Half: " << getHalfNameById(command.gamehalf()).toStdString() << std::endl;

            // Now let's send an placement packet to it

            // First creating an placement command for the blue team
            VSSRef::team_to_ref::VSSRef_Placement placementCommandBlue;
            VSSRef::Frame *placementFrameBlue = new VSSRef::Frame();
            placementFrameBlue->set_teamcolor(VSSRef::Color::BLUE);
            for(int x = 0; x < 3; x++){
                VSSRef::Robot *robot = placementFrameBlue->add_robots();
                robot->set_robot_id(x);
                robot->set_x(0.5);
                robot->set_y(-0.2 + (0.2 * x));
                robot->set_orientation(0.0);
            }
            placementCommandBlue.set_allocated_world(placementFrameBlue);

            // Sending blue
            std::string msgBlue;
            placementCommandBlue.SerializeToString(&msgBlue);
            if(replacerSocket->write(msgBlue.c_str(), msgBlue.length()) == -1){
                std::cout << "[Example] Failed to write to replacer socket: " << replacerSocket->errorString().toStdString() << std::endl;
            }

            // Now creating an placement command for the yellow team
            VSSRef::team_to_ref::VSSRef_Placement placementCommandYellow;
            VSSRef::Frame *placementFrameYellow = new VSSRef::Frame();
            placementFrameYellow->set_teamcolor(VSSRef::Color::YELLOW);
            for(int x = 0; x < 3; x++){
                VSSRef::Robot *robot = placementFrameYellow->add_robots();
                robot->set_robot_id(x);
                robot->set_x(-0.5);
                robot->set_y(-0.2 + (0.2 * x));
                robot->set_orientation(180.0);
            }
            placementCommandYellow.set_allocated_world(placementFrameYellow);

            // Sending yellow
            std::string msgYellow;
            placementCommandYellow.SerializeToString(&msgYellow);
            if(replacerSocket->write(msgYellow.c_str(), msgYellow.length()) == -1){
                std::cout << "[Example] Failed to write to replacer socket: " << replacerSocket->errorString().toStdString() << std::endl;
            }
        }
    }
    return 0;
}
