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

#include "GrafoRRT.h"


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

    Strategy();
    ~Strategy();

    vector<ballPredPos> ballPredMemory; //Vetor de memória com posições passadas
    void predict_ball(fira_message::Ball ball);
    ballPredPos predictedBall; //Inicializado no construtor
    vector<double> predict_vector;

    vector<vector<double>> vRL, VW;

    rrt_graph *rrt = NULL;
    vector<State> *waypopints = NULL;
    bool chave = false;
    int index_rrt = 0;

    //Atributos para zagueiro_cone
    vector<double>* resultante = NULL;
    vector<pair<double,double>>* componentes = NULL;


    int qtdRobos, vrMax;
    double Vmax, Wmax;

    void strategy_blue(fira_message::Robot b0, fira_message::Robot b1,fira_message::Robot b2,
                       fira_message::Robot y0, fira_message::Robot y1,fira_message::Robot y2
                      ,fira_message::Ball ball, const fira_message::Field & field);

    void strategy_blue(Team blue, Team yellow,fira_message::Ball ball, const fira_message::Field & field);

    void strategy_yellow(fira_message::Robot y0, fira_message::Robot y1,
                         fira_message::Robot y2, fira_message::Ball ball, const fira_message::Field & field);

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
    void vaiPara_hotwheels(fira_message::Robot b0, fira_message::Robot b1,fira_message::Robot b2,
                                     fira_message::Robot y0, fira_message::Robot y1,fira_message::Robot y2,
                                     double px, double py,int id);
    bool robo_parede(fira_message::Robot);
    void vaiPara2(fira_message::Robot,double,double,int);
    void sai_robo(fira_message::Robot,fira_message::Robot,double F[]);
    void sai_robo2(fira_message::Robot,fira_message::Robot,double F[]);

    vector<double> inserirRRT(vector<double>,vector<double>,int);
    vector<double> direcao_provavel(ballPredPos, fira_message::Ball);
    void posicionamento(fira_message::Robot, int, bool);
    void goleiro(fira_message::Robot, double, double,int);
    void goleiro_petersson(fira_message::Robot, double, double,int);
    void goleiro2(fira_message::Robot,fira_message::Ball,int);
    void chute(int);
    void zagueiro(fira_message::Robot, double, double,int);
    void zagueiro_cone(Team blue, Team yellow,fira_message::Ball ball,int id);
    void zagueiro2(fira_message::Robot, double, double, int);
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
    double distancia(State,State);

    double limita_velocidade(double, double);

    void RRT(State init, State goal, vector<State> obs_centers,  bool flag = false);
    void RRT(fira_message::Robot rb, vector<double> goal, vector<State> obs_centers,  bool flag = false);


};

#endif // STRATEGY_H


