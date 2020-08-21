#ifndef STRATEGY_H
#define STRATEGY_H
#include <QtNetwork>
#include <stdio.h>
#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"

#include "pb/command.pb.h"
#include "pb/common.pb.h"
#include "pb/packet.pb.h"
#include "pb/replacement.pb.h"

struct ang_err
{
    double fi;
    int flag;
};


//Estrutura para simplificar o uso do preditor
//da bola.
struct ballPredPos
{
    double x;
    double y;
};


class Strategy {
  public:

    Strategy();
    ~Strategy();

    vector<ballPredPos> ballPredMemory; //Vetor de memória com posições passadas
    void predict_ball(fira_message::Ball ball);
    ballPredPos predictedBall; //Inicializado no construtor

    vector<vector<double>> vRL, VW;

    int qtdRobos, vrMax;
    double Vmax, Wmax;

    void strategy_blue(fira_message::Robot b0, fira_message::Robot b1,
                       fira_message::Robot b2, fira_message::Ball ball, const fira_message::Field & field);

    void strategy_yellow(fira_message::Robot y0, fira_message::Robot y1,
                         fira_message::Robot y2, fira_message::Ball ball, const fira_message::Field & field);

    void girarHorario(double,int);
    void girarAntihorario(double,int);
    void andarFrente(double,int);
    void andarFundo(double,int);
    void vaiPara(fira_message::Robot,double,double,int);

    void vaiParaDinamico(fira_message::Robot,double,double,int);

    double controleAngular(double);
    double controleLinear(fira_message::Robot,double,double);

  private:
    double L; //Distância entre roda e centro
    double R; //Raio da roda
    void cinematica_azul(); //transforma V e W em Vr e Vl do time azul
    void cinematica_amarelo(); //transforma V e W em Vr e Vl do time amarelo

    void atualiza_memoria_azul(double, double);
    vector<double> memoria_azul_linear;
    vector<double> memoria_azul_angular;


    ang_err olhar(fira_message::Robot, double, double);
    double distancia(fira_message::Robot,double,double);
    double limita_velocidade(double, double);
};

#endif // STRATEGY_H


