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
                     ,fira_message::Ball ball, const fira_message::Field & field);

    void strategy_yellow(fira_message::Robot y0, fira_message::Robot y1,fira_message::Robot y2,
                         fira_message::Robot b0, fira_message::Robot b1,fira_message::Robot b2,
                         fira_message::Ball ball, const fira_message::Field & field);

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
    void saturacao(double V[]);
    void atualiza_pos(fira_message::Robot b0,fira_message::Robot b1,fira_message::Robot b2,fira_message::Robot y0,fira_message::Robot y1,fira_message::Robot y2);
    void calc_repulsao(fira_message::Robot rb, double F[]);
    void converte_vetor(double V[],double);
    double filtro(double V,int);
    void vaiPara_desviando(fira_message::Robot,double,double,int);
    /*void vaiPara_hotwheels(fira_message::Robot b0, fira_message::Robot b1,fira_message::Robot b2,
                                     fira_message::Robot y0, fira_message::Robot y1,fira_message::Robot y2,
                                     double px, double py,int id);
    bool robo_parede(fira_message::Robot);
    void vaiPara2(fira_message::Robot,double,double,int);
    void sai_robo(fira_message::Robot,fira_message::Robot,double F[]);
    void sai_robo2(fira_message::Robot,fira_message::Robot,double F[]);
    */
    vector<double> inserirRRT(vector<double>,vector<double>,int);
    void goleiro(fira_message::Robot, double, double,int);
    void goleiro_petersson(fira_message::Robot,fira_message::Ball,int);
    void goleiro2(fira_message::Robot,fira_message::Ball,int);
    void chute(int);
    void zagueiro(fira_message::Robot, double, double,int);
    void zagueiro2(fira_message::Robot, double, double, int);
    void zagueiro3(fira_message::Robot rb, double xbola, double ybola, int id);
    void atacante1(fira_message::Robot rb, double xbola, double ybola, int id);
    void calc_repulsao2(fira_message::Robot rb,double px,double py,double F[]);
    void vaiPara_diodo(fira_message::Robot rb,double px, double py, int id);
    void atacante_todos(Team my,Team adv, fira_message::Ball ball, int id, int idzag);
    void zagueiro_todos(Team blue, Team yellow, fira_message::Ball ball, int id);
    //Atributos para zagueiro_cone
    vector<double>* resultante = NULL;
    vector<pair<double,double>>* componentes = NULL;

    //Atributos para atacante_coneLaam
    vector<pair<double,double>>* componentes_2 = NULL;
    vector<double>* resultante_2 = NULL;
    vector<string>* name_vectors = NULL;
    void penalti(fira_message::Robot,fira_message::Ball,int,int);
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


