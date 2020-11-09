#ifndef STRATEGY_H
#define STRATEGY_H
#include <QtNetwork>
#include <stdio.h>
#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"

#include "net/pb/command.pb.h"
#include "net/pb/common.pb.h"
#include "net/pb/packet.pb.h"
#include "net/pb/replacement.pb.h"
#include <time.h>

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

class Team : public std::array<fira_message::Robot,6>
{
public:


    // the constructors
    Team() {}
    Team(fira_message::Robot rb0, fira_message::Robot rb1, fira_message::Robot rb2)
    {

        (*this)[0] = rb0;
        (*this)[1] = rb1;
        (*this)[2] = rb2;

    }
    Team(fira_message::Robot rb0, fira_message::Robot rb1, fira_message::Robot rb2,
         fira_message::Robot rb3, fira_message::Robot rb4, fira_message::Robot rb5)
    {

        (*this)[0] = rb0;
        (*this)[1] = rb1;
        (*this)[2] = rb2;
        (*this)[3] = rb3;
        (*this)[4] = rb4;
        (*this)[5] = rb5;

    }

    //Destructors
    ~Team(){}

};

class Strategy {
  public:

    Strategy(bool time);
    ~Strategy();

    Team *blue = NULL;
    Team *yellow = NULL;

    vector<ballPredPos> ballPredMemory; //Vetor de memória com posições passadas
    void predict_ball(fira_message::Ball ball);
    ballPredPos predictedBall; //Inicializado no construtor

    vector<vector<double>> vRL, VW;

    int qtdRobos, vrMax;
    double Vmax, Wmax;

    int lado;

    void strategy_blue(fira_message::Robot b0, fira_message::Robot b1,fira_message::Robot b2,
                      fira_message::Robot y0, fira_message::Robot y1,fira_message::Robot y2
                     ,fira_message::Ball ball, const fira_message::Field & field,string);

    void strategy_yellow(fira_message::Robot y0, fira_message::Robot y1,fira_message::Robot y2,
                         fira_message::Robot b0, fira_message::Robot b1,fira_message::Robot b2,
                         fira_message::Ball ball, const fira_message::Field & field,string);

    void girarHorario(double,int);
    void girarAntihorario(double,int);
    void andarFrente(double,int);
    void andarFundo(double,int);
    void vaiPara(fira_message::Robot,double,double,int);

    void vaiParaDinamico(fira_message::Robot,double,double,int);
    void vaiParaDinamico2(fira_message::Robot,double,double,int);
    double controleAngular(double);
    double controleLinear(fira_message::Robot,double,double);

    //Alterações Petersson
    double pos_robos[6][2];
    double velocidades[6];
    double velocidades_azul[3][2];
    double velocidades_amarelo[3][2];

    void saturacao(double V[]);
    void atualiza_pos(fira_message::Robot b0,fira_message::Robot b1,fira_message::Robot b2,fira_message::Robot y0,fira_message::Robot y1,fira_message::Robot y2);
    void calc_repulsao(fira_message::Robot rb, double F[]);
    void converte_vetor(double V[],double);
    double filtro(double V,int);
    void vaiPara_desviando(fira_message::Robot,double,double,int);
    vector<double> inserirRRT(vector<double>,vector<double>,int);
    void goleiro_petersson(fira_message::Robot,fira_message::Ball,int);
    void goleiro_petersson2(fira_message::Robot,fira_message::Ball,int);
    void chute(int);
    void zagueiro2(fira_message::Robot, double, double, int);
    void calc_repulsao2(fira_message::Robot rb,double F[]);
    void atacante_todos(Team my,Team adv, fira_message::Ball ball, int id, int idzag);
    void vaiPara_desviando2(fira_message::Robot,double,double,int);

    //Atributos para zagueiro_cone
     vector<pair<double,double>>* componentes = NULL;

    //Atributos para atacante_coneLaam
    vector<pair<double,double>>* componentes_2 = NULL;
    vector<double>* resultante_2 = NULL;
    vector<string>* name_vectors = NULL;
  private:
    double L; //Distância entre roda e centro
    double R; //Raio da roda
    void cinematica_azul(); //transforma V e W em Vr e Vl do time azul
    void cinematica_amarelo(); //transforma V e W em Vr e Vl do time amarelo

    void atualiza_memoria_azul(double, double);
    vector<double> memoria_azul_linear;
    vector<double> memoria_azul_angular;
    void atualiza_memoria_amarelo(double, double);
    vector<double> memoria_amarelo_linear;
    vector<double> memoria_amarelo_angular;

    ang_err olhar(fira_message::Robot, double, double);
    double distancia(fira_message::Robot,double,double);
    double limita_velocidade(double, double);

};



#endif // STRATEGY_H

