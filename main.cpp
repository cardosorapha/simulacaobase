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
#include <string.h>
#include "strategy.h"

#include <iostream>
#include <sstream>

#include "net/vssclient.h"

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

void reposicionar_azul(double x[],double y[],double ori[],QUdpSocket *replacerSocket){
    // First creating an placement command for the blue team
    VSSRef::team_to_ref::VSSRef_Placement placementCommandBlue;
    VSSRef::Frame *placementFrameBlue = new VSSRef::Frame();
    placementFrameBlue->set_teamcolor(VSSRef::Color::BLUE);

    for(int i = 0; i < 3; i++){
        VSSRef::Robot *robot = placementFrameBlue->add_robots();
        robot->set_robot_id(static_cast<uint32_t>(i));
        robot->set_x(x[i]);
        robot->set_y(y[i]);
        robot->set_orientation(ori[i]);
    }

    placementCommandBlue.set_allocated_world(placementFrameBlue);

    // Sending blue
    std::string msgBlue;
    placementCommandBlue.SerializeToString(&msgBlue);
    if(replacerSocket->write(msgBlue.c_str(), static_cast<qint64>(msgBlue.length())) == -1){
        std::cout << "[Example] Failed to write to replacer socket: " << replacerSocket->errorString().toStdString() << std::endl;
    }
}

void reposicionar_amarelo(double x[],double y[],double ori[],QUdpSocket *replacerSocket){
    // Now creating an placement command for the yellow team
    VSSRef::team_to_ref::VSSRef_Placement placementCommandYellow;
    VSSRef::Frame *placementFrameYellow = new VSSRef::Frame();
    placementFrameYellow->set_teamcolor(VSSRef::Color::YELLOW);

    for (int i = 0; i<3 ;i++){
        VSSRef::Robot *robot = placementFrameYellow->add_robots();
        robot->set_robot_id(static_cast<uint32_t>(i));
        robot->set_x(x[i]);
        robot->set_y(y[i]);
        robot->set_orientation(ori[i]);
    }

    placementCommandYellow.set_allocated_world(placementFrameYellow);

    // Sending yellow
    std::string msgYellow;
    placementCommandYellow.SerializeToString(&msgYellow);
    if(replacerSocket->write(msgYellow.c_str(), static_cast<qint64>(msgYellow.length())) == -1){
        std::cout << "[Example] Failed to write to replacer socket: " << replacerSocket->errorString().toStdString() << std::endl;
    }
}

using namespace std;

int main(int argc, char *argv[]){

    //SETUP VISAO
    string IP;
    string vision;
    string command;
    string referee_port;
    string replacer_port;
    string campo;

    if(argc == 7){ // se existir entrada via terminal
        IP = argv[1];
        vision = argv[2];
        command = argv[3];
        referee_port = argv[4];
        replacer_port = argv[5];
        campo = argv[6];
    }else{         //se não existir executa os valores abaixo
        IP = "224.0.0.1";
        vision = "10002";
        command = "20011";
        referee_port = "10003";
        replacer_port = "10004";
        campo = "amarelo";
    }
    //converte de string pra inteiro
    stringstream aux(vision);
    int visao = 0;
    aux >> visao;
    //converte de string pra inteiro
    stringstream aux2(command);
    int comando = 0;
    aux2 >> comando;
    //detecta se vai jogar com os dois robos ou qual dos times irão jogar
    bool my_robots_are_yellow = false;
    bool my_robots_are_blue = false;
    bool ambos;

    if (campo == "azul"){
        my_robots_are_yellow = false;
        my_robots_are_blue = true;
        ambos = false;
    }else if (campo == "amarelo"){
            my_robots_are_yellow = true;
            my_robots_are_blue = false;
            ambos = false;
    }else if (campo == "ambos"){
            my_robots_are_yellow = true;
            my_robots_are_blue = true;
            ambos = true;
    }else{
        printf("ERRO ESCOLHA DE CAMPO");
        exit(EXIT_FAILURE);
    }

    //SETUP JUIZ
    QUdpSocket *replacerSocket = new QUdpSocket();
    QUdpSocket *refereeClient = new QUdpSocket();
    //VSSClient *client = new VSSClient(REFEREE_PORT, IP);

    // Performing connection to send Replacer commands
    if(replacerSocket->isOpen())
        replacerSocket->close();

    QString UDP = QString::fromUtf8(IP.c_str());
    //converte de string pra inteiro
    stringstream aux3(replacer_port);
    int porta_replacer = 0;
    aux3 >> porta_replacer;

    replacerSocket->connectToHost(UDP, porta_replacer, QIODevice::WriteOnly, QAbstractSocket::IPv4Protocol);
   // replacerSocket->connectToHost(UDP_ADDRESS, REPLACER_PORT, QIODevice::WriteOnly, QAbstractSocket::IPv4Protocol);
    std::cout << "Connected to REPLACER socket in port " << porta_replacer << " and address = " << IP << ".\n";

    //converte de string pra inteiro
    stringstream aux4(referee_port);
    int porta_juiz = 0;
    aux4 >> porta_juiz;


    // Perfoming connection to receive Referee commands binding port
    if(refereeClient->bind(QHostAddress::AnyIPv4, porta_juiz, QUdpSocket::ShareAddress) == false){
        std::cout << "[ERROR] Failed to bind referee client =(" << std::endl;
        exit(-1);
    }
    // connecting to multicast group in UDP_ADDRESS
    if(refereeClient->joinMulticastGroup(QHostAddress(UDP)) == false){
        std::cout << "[ERROR] Failed to join VSSReferee multicast group =(" << std::endl;
        exit(-1);
    }
    std::cout << "[Example] Connected to REFEREE socket in port " << porta_juiz << " and address = " << IP << ".\n";


    //SETUP DO JOGO
    // the ip address need to be in the range 224.0.0.0 through 239.255.255.255
    RoboCupSSLClient visionClient(IP, visao);
    visionClient.open(false);

    GrSim_Client commandClient("127.0.0.1", comando);

    fira_message::sim_to_ref::Environment packet;

    //inicialização da classe estratégia

    //inicializa duas estrategias uma para cada time em caso de ambos os times serem utilizados
    Strategy estrategia_amarela(my_robots_are_yellow);
    Strategy estrategia_azul(my_robots_are_blue);

    //inicializa estrategia azul ou amarela
    Strategy estrategia(my_robots_are_yellow);

    string ultimo_comando;
    string cor;
    string quadrante;
    //Lendo o JUIZ
    VSSRef::ref_to_team::VSSRef_Command send_command;

    bool received = false;
    char *buffer = new char[65535];
    long long int packetLength = 0;

    while(true) {

        while(refereeClient->hasPendingDatagrams()){

            // Parse message to protobuf
            packetLength = refereeClient->readDatagram(buffer, 65535);

            if(send_command.ParseFromArray(buffer, int(packetLength)) == false){
                std::cout << "[ERROR] Referee send_command parsing error =(" << std::endl;
                exit(-1);
            }

            ultimo_comando = getFoulNameById(send_command.foul()).toStdString();
            cor = getTeamColorNameById(send_command.teamcolor()).toStdString();
            quadrante = getQuadrantNameById(send_command.foulquadrant()).toStdString();

            cout<<ultimo_comando<<endl<<cor<<endl<<quadrante<<endl;

           //ENVIA REPOSICIONAMENTO AMARELO
           if(my_robots_are_yellow){

               if(ultimo_comando == "FREE_BALL"){
                     double x[3] = {0.7 , 0.3 , -0.3};
                     double y[3] = {0 , 0 , 0.3};
                     double ori[3] = {90 , 0 , 0};

                     if(quadrante == "QUADRANT 1"){
                         x[2] = 0.53;
                         y[2] = 0.3;
                         ori[2] = 135;
                     }
                     if(quadrante == "QUADRANT 4"){
                         x[2] = 0.53;
                         y[2] = -0.3;
                         ori[2] = 225;
                     }
                     if(quadrante == "QUADRANT 2"){
                         x[2] = -0.18;
                         y[2] = 0.4;
                     }
                     if(quadrante == "QUADRANT 3"){
                         x[2] = -0.18;
                         y[2] = -0.4;
                     }
                     reposicionar_amarelo(x,y,ori,replacerSocket);
               }

               if(ultimo_comando == "KICKOFF"){
                    if(cor == "YELLOW"){
                        double x[3] = {0.7,0.3,0.05};
                        double y[3] = {0,0,0.05};
                        double ori[3] = {90,0,-135};
                        reposicionar_amarelo(x,y,ori,replacerSocket);
                    }
                    if(cor == "BLUE"){
                        double x[3] = {0.7,0.3,0.5};
                        double y[3] = {0,0,0};
                        double ori[3] = {90,0,0};
                        reposicionar_amarelo(x,y,ori,replacerSocket);
                    }
               }

               if(ultimo_comando == "GOAL_KICK"){
                   if(cor == "BLUE"){
                       double x[3] = {0.7,0.3,0.3};
                       double y[3] = {0,-0.3,0.3};
                       double ori[3] = {0,0,0};
                       reposicionar_amarelo(x,y,ori,replacerSocket);
                   }
                   if(cor == "YELLOW"){
                       double x[3] = {0.7,0.3,0.3};
                       double y[3] = {0,-0.3,0.3};
                       double ori[3] = {180,0,0};
                       reposicionar_amarelo(x,y,ori,replacerSocket);
                   }
               }

               if(ultimo_comando == "PENALTY_KICK"){
                   if(cor == "BLUE"){
                       double x[3] = {0.7,-0.05,-0.05};
                       double y[3] = {0,-0.3,0.3};
                       double ori[3] = {90,0,0};
                       reposicionar_amarelo(x,y,ori,replacerSocket);
                   }
                   if(cor == "YELLOW"){
                       double x[3] = {0.7,0.05,-0.3};
                       double y[3] = {0,0,0};
                       double ori[3] = {90,0,0};
                       reposicionar_amarelo(x,y,ori,replacerSocket);
                   }
               }
            }
             //ENVIA REPOSICIONAMENTO AZUL
            if(my_robots_are_blue){ //AZUL
                 if(ultimo_comando == "FREE_BALL"){
                       double x[3] = {-0.7,-0.3,-0.3};
                       double y[3] = {0,0,0.3};
                       double ori[3] = {90,0,0};

                       if(quadrante == "QUADRANT 1"){
                           x[2] = 0.18;
                           y[2] = 0.4;
                       }
                       if(quadrante == "QUADRANT 4"){
                           x[2] = 0.18;
                           y[2] = -0.4;
                       }
                       if(quadrante == "QUADRANT 2"){
                           x[2] = -0.53;
                           y[2] = 0.3;
                           ori[2] = 45;
                       }
                       if(quadrante == "QUADRANT 3"){
                           x[2] = -0.53;
                           y[2] = -0.3;
                           ori[2] = -45;
                       }
                       reposicionar_azul(x,y,ori,replacerSocket);
               }

               if(ultimo_comando == "KICKOFF"){
                   if(cor == "BLUE"){
                       double x[3] = {-0.7,-0.3,-0.05};
                       double y[3] = {0,0,0.05};
                       double ori[3] = {90,0,-45};
                       reposicionar_azul(x,y,ori,replacerSocket);
                   }
                   if(cor == "YELLOW"){
                       double x[3] = {-0.7,-0.3,-0.5};
                       double y[3] = {0,0,0};
                       double ori[3] = {90,0,0};
                       reposicionar_azul(x,y,ori,replacerSocket);
                   }
               }

               if(ultimo_comando == "GOAL_KICK"){
                   if(cor == "BLUE"){
                       double x[3] = {-0.7,-0.3,-0.3};
                       double y[3] = {0,-0.3,0.3};
                       double ori[3] = {90,0,0};
                       reposicionar_azul(x,y,ori,replacerSocket);
                   }
                   if(cor == "YELLOW"){
                       double x[3] = {-0.7,-0.3,-0.3};
                       double y[3] = {0,-0.3,0.3};
                       double ori[3] = {90,0,0};
                       reposicionar_azul(x,y,ori,replacerSocket);
                   }
               }

               if(ultimo_comando == "PENALTY_KICK"){
                   if(cor == "BLUE"){
                       double x[3] = {-0.7,-0.05,0.3};
                       double y[3] = {0,0,0};
                       double ori[3] = {90,0,0};
                       reposicionar_azul(x,y,ori,replacerSocket);
                   }
                   if(cor == "YELLOW"){
                       double x[3] = {-0.7,0.05,0.05};
                       double y[3] = {0,-0.3,0.3};
                       double ori[3] = {90,0,0};
                       reposicionar_azul(x,y,ori,replacerSocket);
                   }
               }
           }
        }
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

                //printf("-[Geometry Data]-------\n");
                const fira_message::Field & field = packet.field();

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

                    estrategia_amarela.strategy_yellow(y0,y1,y2,b0,b1,b2,ball,field,ultimo_comando);
                    estrategia_azul.strategy_blue(b0,b1,b2,y0,y1,y2,ball,field,ultimo_comando);

                    //Enviando velocidades para os robos amarelos
                    for(int i = 0;i < estrategia_amarela.qtdRobos;i++)
                        commandClient.sendCommand(estrategia_amarela.vRL[i][1],estrategia_amarela.vRL[i][0],my_robots_are_yellow,i);

                    //Enviando velocidades para os robos azuis
                    for(int i = 0;i < estrategia_azul.qtdRobos;i++)
                        commandClient.sendCommand(estrategia_azul.vRL[i][1],estrategia_azul.vRL[i][0],!my_robots_are_yellow,i);

                }else{
                    estrategia.atualiza_pos(b0,b1,b2,y0,y1,y2);

                    if(my_robots_are_yellow){
                        estrategia.strategy_yellow(y0,y1,y2,b0,b1,b2,ball,field,ultimo_comando);
                    }else{
                        estrategia.strategy_blue(b0,b1,b2,y0,y1,y2,ball,field,ultimo_comando);
                    }
                    //Enviando velocidades para os robos
                    for(int i = 0;i < estrategia.qtdRobos;i++)
                        commandClient.sendCommand(estrategia.vRL[i][1],estrategia.vRL[i][0],my_robots_are_yellow,i);

                }
            }
        }
    }
    return 0;
}
