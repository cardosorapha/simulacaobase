#ifndef GRAFORRT_H
#define GRAFORRT_H



//OUTROS
#include <iostream>
#include <vector>
#include <stdio.h>
#include <time.h>
#include <list>
#include <algorithm>
#include <utility>
#include <iterator>
#include <array>
#include <bits/stdc++.h>




using namespace std;

enum sit{Advanced,Reached,Trapped};

class State : public std::array<double, 3>
{
public:

    // dimension of space (or "k" of k-d tree)
    // KDTree class accesses this member
    static const int DIM = 3;

    // the constructors
    State() {}
    State(double x, double y, double theta = 0)
    {
        (*this)[0] = x;
        (*this)[1] = y;
        (*this)[2] = theta;

        this->x = x;
        this->y = y;
        this->theta = theta;
    }


    // conversion to OpenCV Point2d
    //operator cv::Point() const { return cv::Point((*this)[0], (*this)[1]); }

    double x,y,theta;

};

class rrt_graph
{
private:
    vector<State> nodes;
    vector<pair<int,double>> pesos;
    vector<State> optimal_nodes;   //Path Smoothing
    vector<list<int>*> adj;   //Lista de ajacências
    int numNodes;
    State Goal;

    //Métodos auxiliares
    static bool sortbyfirst(const pair<int,double> &a, const pair<int,double> &b)
    {
        return (a.second < b.second);
    }

public:

    //Construtores
    rrt_graph()
    {
        this->numNodes = 0;
    }
    rrt_graph(State init,State _goal)
    {
        x_new = init;
        Goal = _goal;
        pesos.push_back(make_pair(0,sqrt(pow(_goal.x-init.x,2.0)+pow(_goal.y-init.y,2.0))));
        nodes.push_back(init);
        adj.push_back(new list<int>);
        this->numNodes = 1;
    }
    rrt_graph(State init)
    {
        pesos.push_back(make_pair(0,0.0));
        nodes.push_back(init);
        adj.push_back(new list<int>);
        this->numNodes = 1;
    }

    //Destrutor
    ~rrt_graph()
    {
        for(int i = 0; i<numNodes;i++)
        {
            delete adj[i];
        }
    }

    //Funções Add
    void add_vertice(State node)
    {
        pesos.push_back(make_pair(numNodes,sqrt(pow(Goal.x-node.x,2.0)+pow(Goal.y-node.y,2.0))));
        nodes.push_back(node);
        adj.push_back(new list<int>);
        numNodes++;
    }
    void add_aresta(int index_source, int index_target)
    {
        adj[index_source]->push_back(index_target);
    }

    //Funções Get

    State GetNodeState(int index_node) const
    {
        return nodes[index_node];
    }
    vector<State> GetOptimaNodes() const
    {
        return optimal_nodes;
    }
    vector<double> GetNodeVecDir(int index_node) const
    {
        float x = cos(GetNodeState(index_node).theta);
        float y = sin(GetNodeState(index_node).theta);

        float norm = sqrt((GetNodeState(index_node).x*GetNodeState(index_node).x)
                            +(GetNodeState(index_node).y*GetNodeState(index_node).y));

        vector<double> v;
        v.push_back(norm*x);
        v.push_back(norm*y);
        return v;
    }
    list<int> GetNodeNeighbors(int index_node) const
    {
        return *adj[index_node];
    }
    int GetNumNodes() const
    {
        return numNodes;
    }


//Implementações Do movimento da RRT - São as implementações que variam com a técnica ultilizda

    //Método 'random_state' - RRT clássica
    State random_state(double width, double height)
    {
        return State(static_cast <double> (rand())/(static_cast <double> (RAND_MAX/width)),
                        static_cast <double> (rand())/(static_cast <double> (RAND_MAX/height)));
    }
    //Sobrecarda de 'random_state' - RRT-Goalbias
    State random_state(double width, double height, State goal, float epislon)
    {
        State aux;
        if(((float)rand()/RAND_MAX) < epislon)
            aux = State(static_cast <double> (rand())/(static_cast <double> (RAND_MAX/width)),
                        static_cast <double> (rand())/(static_cast <double> (RAND_MAX/height)));
        else
            aux = goal;

        return aux;
    }
    //Sobrecarda de 'random_state' - RRT-Goalbias e Horizonte amostral
    State random_state(State init, State goal, double epislon, double raio)
    {

        double randy = static_cast <double> (2*raio*(rand())/(static_cast <double> (RAND_MAX)) - raio);
        double randx;
        if(goal.x < init.x)
            randx = static_cast <double> (raio*(rand())/(static_cast <double> (RAND_MAX)) - raio);
        else
            randx = static_cast <double> (raio*(rand())/(static_cast <double> (RAND_MAX)));

        State aux;
        if(((float)rand()/RAND_MAX) < epislon)
            aux = State(init.x + randx, init.y + randy);
        else
            aux = goal;

        return aux;
    }
    //Sobrecarda de 'random_state' - ERRT
    State random_state(double width, double height, State goal, vector<State> waypointCahe ,float epislon,float delta)
    {
        float p = (float)rand()/RAND_MAX;
        int i = rand() % waypointCahe.size();

        State aux;

        if((0 < p) &&(p < epislon))
            return goal;
        else if((epislon < p)&&(p < epislon + delta))
            return waypointCahe[i];
        else if((epislon + delta < p)&&(p < 1))
            return aux = State(static_cast <double> (rand())/(static_cast <double> (RAND_MAX/width)),
                               static_cast <double> (rand())/(static_cast <double> (RAND_MAX/height)));
        return aux;
    }

    //Sobrecarda de 'random_state' - ERRT e Horizonte amostral
    State random_state(State init, State goal,double epislon,double delta,double raio, vector<State> *waypointCahe = NULL )
    {
        double randy = static_cast <double> (2*raio*(rand())/(static_cast <double> (RAND_MAX)) - raio);
        double randx;
        if(goal.x < init.x)
            randx = static_cast <double> (raio*(rand())/(static_cast <double> (RAND_MAX)) - raio);
        else
            randx = static_cast <double> (raio*(rand())/(static_cast <double> (RAND_MAX)));

        float p = (float)rand()/RAND_MAX;
        int i = rand() % waypointCahe->size();

        State aux;

        if((0 < p) &&(p < epislon))
            return goal;
        else if((epislon < p)&&(p < epislon + delta))
            return (*waypointCahe)[i];
        else if((epislon + delta < p)&&(p < 1))
            return aux = State(init.x + randx, init.y + randy);

        return aux;
    }


    //Método 'nearest_neighbor' retorna o id do nó mas próximo do ponto 'x_rand'
    int nearest_neighbor(State x_rand)
    {
        double min_dist = 99999.9;
        int nearest_node = 0;
        for (int node = 0; node < GetNumNodes() ; node++)
        {
            double sum = 0.0, dist;

            sum = pow(GetNodeState(node).x - x_rand.x, 2.0)
                    + pow(GetNodeState(node).y - x_rand.y, 2.0);

            dist = sqrt(sum);

            if (dist < min_dist)
            {
                min_dist = dist;
                nearest_node = node;
            }

        }

        return nearest_node;
    }



    //Método 'move' determina como a arvore vai expandir - Função de transição de Estado
    State move(State source, State target)
    {
        //Encontrar orientação
        target.x = target.x - source.x;
        target.y = target.y - source.y;
        double angle = atan2(target.x,target.y);
        if(angle < 0)
            angle = angle + 2*M_PI;

       // cout << angle << endl;

        State delta;

        delta.x = sin(angle);
        delta.y = cos(angle);

       // cout <<"delta antes: " << delta << endl;


        //Movimento na direção X
        if((0.0 < delta.x)&&(delta.x < 0.3535))
            delta.x = 0;
        else if((0.3535 < delta.x)&&(delta.x < 1))
            delta.x = 0.03;
        else if((-0.3535 < delta.x)&&(delta.x < 0))
            delta.x = 0;
        else if((-1 < delta.x)&&(delta.x < -0.3535))
            delta.x = -0.03;

        //Movimento na direção Y
        if((0.0 < delta.y)&&(delta.y < 0.3535))
            delta.y = 0;
        else if((0.3535 < delta.y)&&(delta.y < 1))
            delta.y = 0.03;
        else if((-0.3535 < delta.y)&&(delta.y < 0))
            delta.y = 0;
        else if((-1 < delta.y)&&(delta.y < -0.3535))
            delta.y = -0.03;

        //cout <<"delta depois: " << delta << endl;


        return delta;

    }


    //Método 'new_state' implementa o método 'move' para retornar o ponto x_new
    State new_state(State x_near, State x_rand)
    {
        return State(x_near.x + move(x_near, x_rand).x, x_near.y + move(x_near, x_rand).y);
    }


    //Método 'Extend' - Testa a expansão da RRT for RRT-CONNECT
    State x_new;
    int Extend(State x_rand, vector<State> center_obs, double raio)
    {
        int nearest_node = nearest_neighbor(x_rand);
        State x_near = GetNodeState(nearest_node);

        x_new = new_state(x_near,x_rand);

        if(obs_detect(x_new,center_obs,raio))
         {
             //Verifica se x_new já é um nó do grafo
              bool existis_node = false;
              for (int i = 0;i < GetNumNodes();i++)
              {
                 if(x_new == GetNodeState(i))
                  {
                        existis_node = true;
                        return Reached;
                  }
               }
               if(existis_node == false)
               {
                  //Adiciona vértice à arvore
                  add_vertice(x_new);
                  //Adiciona Aresta
                  add_aresta(nearest_node,(GetNumNodes()-1));

                  /* if(x_new == Goal)
                        return Reached;*/

                   return Advanced;
                }

         }
        return Trapped;
    }
    //Método 'Extend_SH' - Testa a expansão da RRT para RRT com Horizonte amostral
    int Extend_SH(State x_rand, vector<State> center_obs, double raio_obs, double raio = 0.04)
    {

        int nearest_node = nearest_neighbor(x_rand);
        State x_near = GetNodeState(nearest_node);

        x_new = new_state(x_near,x_rand);

        double dist = sqrt(pow(x_new.x - nodes.begin()->x,2.0)+pow(x_new.y - nodes.begin()->y,2.0));

        if(obs_detect(x_new,center_obs,raio_obs))
         {
             //Verifica se x_new já é um nó do grafo
              bool existis_node = false;
              for (int i = 0;i < GetNumNodes();i++)
              {
                 if(x_new == GetNodeState(i))
                  {
                        existis_node = true;
                        return Reached;

                  }
               }
               if(existis_node == false && dist < raio)
               {
                  //Adiciona vértice à arvore
                  add_vertice(x_new);
                  //Adiciona Aresta
                  add_aresta(nearest_node,(GetNumNodes()-1));

                  return Advanced;
                }

         }

        return Trapped;


    }
    //Método 'Connect' - Testa a expansão da RRT for RRT-CONNECT
    int Connect(State x_rand, vector<State> center_obs, double raio)
    {
        int S;
        do
        {
            S = Extend(x_rand,center_obs,raio);

        }while (S==Advanced);

        return S;
    }
    //Método 'connect_test' testa a conecção entre os nós e retorna o nó de destino se a conecção for válida - Mapa "aberto"
    bool test_connect(State source, State target,vector<State> center_obs,double raio)
    {
        //vetor 'V'
        State V = State(target.x-source.x,target.y-source.y);
        //Norma do vetor 'V'
        double norm_V = sqrt(pow(V.x,2.0)+pow(V.y,2.0));

        for(int i = 0; i<(int)center_obs.size();i++)
        {
            //vetor 'AP'
            State AP = State(center_obs[i].x-source.x,center_obs[i].y-source.y);

            //Norma do produto vetorial VxAP
            double dist = abs((V.x*AP.y)-(V.y*AP.x))/norm_V;

            if(dist < raio)
                return false;


        }
        return true;
    }



    //Método 'connect_test' testa a conecção entre os nós e retorna o nó de destino se a conecção for válida - Mapa "aberto"
    State connect_test(State source, State target,vector<State> center_obs,double raio)
    {
        //vetor 'V'
        State V = State(target.x-source.x,target.y-source.y);
        //Norma do vetor 'V'
        double norm_V = sqrt(pow(V.x,2.0)+pow(V.y,2.0));

        for(int i = 0; i<(int)center_obs.size();i++)
        {
            //vetor 'AP'
            State AP = State(center_obs[i].x-source.x,center_obs[i].y-source.y);

            //Norma do produto vetorial VxAP
            double dist = abs((V.x*AP.y)-(V.y*AP.x))/norm_V;

            if(dist < raio)
                return source;


        }
        return target;
    }

    //Método 'smooth_path' retorna um vetor de pontos para tragetória otimizda - Mapa "aberto"
    vector<State> smooth_path( vector<State> center_obs, double raio)
    {

        State _init = GetNodeState(0);
        optimal_nodes.push_back(_init);
        _init = connect_test(_init,Goal,center_obs,raio);
        if(_init==Goal)
            optimal_nodes.push_back(_init);

        vector<pair<int,double>> pesos_copy;
        pesos_copy.assign(pesos.begin(),pesos.end());

        std::sort(pesos_copy.begin(),pesos_copy.end(),sortbyfirst);

        int limite = pesos_copy.size();

        for(int node = 0; node <limite; node++)
        {
            State aux = connect_test(_init,nodes[pesos_copy[node].first],center_obs,raio);
            if(aux != _init)
            {
                _init = aux;
                optimal_nodes.push_back(_init);
                if(_init == Goal)
                    break;


                limite = node;
                node = 0;
            }

        }

        return optimal_nodes;

    }

    //Método 'obs_detect' retorna um booleano 'true' se o estado estiver contido no c_cobs - Mapa "aberto"
    bool obs_detect(State current, vector<State> center_obs, double raio/*alcane*/)
    {
        for(int i = 0;i < (int)center_obs.size();i++)
        {
            double dist = sqrt(pow(current.x-center_obs[i].x,2.0)
                                +  pow(current.y-center_obs[i].y,2.0));
            if(dist <= raio)
                return false;

        }

        return true;
    }


};


#endif // GRAFORRT_H
