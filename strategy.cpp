#include "strategy.h"
#include "GrafoRRT.h"

Strategy::Strategy(bool time)
{
    L = 0.04; //Distância entre roda e centro
    R = 0.02; //Raio da roda

    vector<double> aux{0,0};

    for(int i = 0; i < 5 ; i++){
       velocidades[i] = 0;
    }

    qtdRobos = 3;
    vrMax = 125;
    Vmax = 2.5;
    Wmax = 62.5;

    for(int i = 0; i < qtdRobos; i++)
    {
        vRL.push_back(aux);
        VW.push_back(aux);
    }

    //Inicialização do vetor de preditor
    int N = 5;
    ballPredPos temp;
    temp.x = 0;
    temp.y = 0;

    for(int i = 0; i < N; i++)
    {
        ballPredMemory.push_back(temp);
    }

    predictedBall = temp;

    //Inicialização dos vetores de memória PID
    int N_mem = 5;

    for(int i=0; i < N_mem; i++)
    {
        memoria_azul_linear.push_back(0);
        memoria_azul_angular.push_back(0);
    }

    //verifica se foi criado um time amarelo
    if(time == true){
        lado = -1;
    }else{//ou time azul
        lado = 1;
    }
}

//Está fazendo muito pouca diferença, talvez deva diminuir o tempo
//de amostragem.
void Strategy::predict_ball(fira_message::Ball ball)
{

    // Depende de quantas foram inicializadas no construtor
    // Retornarei um vetor de 3 pontos, tais que os primeiros elementos são futuros mais próximos
    int N = 0;
    float a = 0;     // Começo da pseudoinversão (A'A)^(-1)
    float theta = 0; //Guarda o termo do preditor
    float temp = 0;

    int futureTime = 50; //Representa quantas iterações no futuro o robô vai predizer a bola
    //Atenção: quanto mais no futuro, mais errado, então imagine esse valor como um ganho de preditor
    //tal que MAIOR = mais pra frente, MENOR = menos pra frente no tempo.

    N = this->ballPredMemory.size()-1; // indice do ultimo elemento

    ballPredPos result; //Resultado será guardado aqui.

    //Atualização do vetor de memória
    ballPredPos ballUpdate;
    ballUpdate.x = ball.x();
    ballUpdate.y = ball.y();
    ballPredMemory.push_back(ballUpdate); //Adicionando novo elemento à memória
    ballPredMemory.erase(ballPredMemory.begin()+0); //Removendo último valor da memória

    //Primeiro para a posição x.
    for (int m = 0; m < (N-1);m++)
       a = a + pow(this->ballPredMemory[m].x,2);
    a = 1/(a + 0.001);

    for(int m = 0; m < (N-1);m++)
       theta = theta + a * this->ballPredMemory[m].x * this->ballPredMemory[m+1].x;
    theta = pow(theta,futureTime);
    result.x = theta * ball.x();

    //Agora pra posicao y.
    a = 0;
    theta = 0;

    for (int m = 0; m < (N-1);m++)
       a = a + pow(this->ballPredMemory[m].y,2);
    a = 1/(a + 0.001);

    for(int m = 0; m < (N-1);m++)
       theta = theta + a * this->ballPredMemory[m].y * this->ballPredMemory[m+1].y;
    theta = pow(theta,futureTime);
    result.y = theta * ball.y();

    //"Retorno" da função.
    this->predictedBall = result;
}

void Strategy::strategy_blue(fira_message::Robot b0, fira_message::Robot b1,fira_message::Robot b2,
                             fira_message::Robot y0, fira_message::Robot y1,fira_message::Robot y2,
                             fira_message::Ball ball, const fira_message::Field & field,string sit_juiz)
{
   Team blue(b0,b1,b2);
   Team yellow(y0,y1,y2);

    double dist1 = sqrt(pow(b1.x()-ball.x(),2)+pow(b1.y()-ball.y(),2));
    double dist2 = sqrt(pow(b2.x()-ball.x(),2)+pow(b2.y()-ball.y(),2));

    if(sit_juiz == "GAME_ON"){
         goleiro_petersson2(b0,ball,0);
         if(ball.x() > -0.1){
             if (dist1 > dist2){
                 zagueiro2(b1,ball.x(),ball.y(),1);
                 atacante_todos(blue,yellow,ball,2,1);
             }else{
                 zagueiro2(b2,ball.x(),ball.y(),2);
                 atacante_todos(blue,yellow,ball,1,2);
             }
         }else{
             zagueiro2(b1,ball.x(),ball.y(),1);
             atacante_todos(blue,yellow,ball,2,1);
         }
    }else{
        andarFrente(0,0);
        andarFrente(0,1);
        andarFrente(0,2);
    }

   cinematica_azul();
}

void Strategy::strategy_yellow(fira_message::Robot y0, fira_message::Robot y1,fira_message::Robot y2,
                               fira_message::Robot b0, fira_message::Robot b1,fira_message::Robot b2,
                               fira_message::Ball ball, const fira_message::Field & field,string sit_juiz)
{
    Team blue(b0,b1,b2);
    Team yellow(y0,y1,y2);

    double dist1 = sqrt(pow(y1.x()-ball.x(),2)+pow(y1.y()-ball.y(),2));
    double dist2 = sqrt(pow(y2.x()-ball.x(),2)+pow(y2.y()-ball.y(),2));

    if(sit_juiz == "GAME_ON"){

        goleiro_petersson2(y0,ball,0);
         if(ball.x() < 0.1){
             if (dist1 > dist2){
                 zagueiro2(y1,ball.x(),ball.y(),1);
                 atacante_todos(yellow,blue,ball,2,1);
             }else{
                 zagueiro2(y2,ball.x(),ball.y(),2);
                 atacante_todos(yellow,blue,ball,1,2);
             }
         }else{
             zagueiro2(y1,ball.x(),ball.y(),1);
             atacante_todos(yellow,blue,ball,2,1);
         }

    }else{
        andarFrente(0,0);
        andarFrente(0,1);
        andarFrente(0,2);
    }

    cinematica_amarelo();
}

//Calcula as velocidades a serem enviadas ao robô, utilizando cinematica inversa
void Strategy::cinematica_azul()
{
    double k = 0.9;

    for(int i = 0; i < qtdRobos; i++)
    {
        vRL[i][0] = (VW[i][0]+VW[i][1]*L)/R;
        vRL[i][1] = (VW[i][0]-VW[i][1]*L)/R;

        vRL[i][0] = limita_velocidade(vRL[i][0],vrMax);
        vRL[i][1] = limita_velocidade(vRL[i][1],vrMax);

        vRL[i][0] = velocidades_azul[i][0]*(1 - k) + vRL[i][0]*k;
        vRL[i][1] = velocidades_azul[i][1]*(1 - k) + vRL[i][1]*k;

        velocidades_azul[i][0] = vRL[i][0]*k;
        velocidades_azul[i][1] = vRL[i][1]*k;

    }
}

void Strategy::andarFrente(double vel, int id)
{
    double Vaux = vel/vrMax;
    VW[id][0] = Vmax*Vaux;
    VW[id][1] = 0;
}
void Strategy::cinematica_amarelo()
{
    double k = 0.8;
    for(int i = 0; i < qtdRobos; i++)
    {
        vRL[i][0] = (VW[i][0]+VW[i][1]*L)/R;
        vRL[i][1] = (VW[i][0]-VW[i][1]*L)/R;

        vRL[i][0] = limita_velocidade(vRL[i][0],vrMax);
        vRL[i][1] = limita_velocidade(vRL[i][1],vrMax);

        vRL[i][0] = velocidades_amarelo[i][0]*(1 - k) + vRL[i][0]*k;
        vRL[i][1] = velocidades_amarelo[i][1]*(1 - k) + vRL[i][1]*k;

        velocidades_amarelo[i][0] = vRL[i][0]*k;
        velocidades_amarelo[i][1] = vRL[i][1]*k;
    }
}

void Strategy::andarFundo(double vel, int id)
{
    double Vaux = vel/vrMax;
    VW[id][0] = -Vmax*Vaux;
    VW[id][1] = 0;
}

void Strategy::girarHorario(double vel,int id)
{
    double Waux = vel/vrMax;
    VW[id][0] = 0;
    VW[id][1] = Wmax*Waux;
}

void Strategy::girarAntihorario(double vel,int id)
{
    double Waux = vel/vrMax;
    VW[id][0] = 0;
    VW[id][1] = -Waux*Wmax;
}

void Strategy::vaiPara(fira_message::Robot rb, double px, double py, int id)
{
    VW[id][0] = controleLinear(rb,px,py);
    ang_err angulo = olhar(rb, px, py);
    VW[id][1] = controleAngular(angulo.fi);
}

void Strategy::vaiParaDinamico(fira_message::Robot rb, double px, double py, int id)
{
    ang_err angulo = olhar(rb, px, py);
    double erro_angular = angulo.fi; //de -180 a 180
    double erro_linear = angulo.flag*distancia(rb,px,py);

    double V = 0;
    double W = 0;

    double Kp_l = 10;
    double Ki_l = 5;
    double Kd_l = 0.01;

    double Kp_a = 0.15;
    double Ki_a = 0.02;
    double Kd_a = 0.01;

    double temp_integral_l = 0;
    double temp_integral_a = 0;

    if(lado == 1){
        for (int i = 0; i < (int)memoria_azul_linear.size(); i++)
        {
            temp_integral_l = temp_integral_l + memoria_azul_linear.at(i);
        }

        for (int i = 0; i < (int)memoria_azul_angular.size(); i++)
        {
            temp_integral_a = temp_integral_a + memoria_azul_angular.at(i);
        }

        V = Kp_l*erro_linear + Ki_l*temp_integral_l + Kd_l*(erro_linear-memoria_azul_linear.at(0));

        W = Kp_a*erro_angular + Ki_a*temp_integral_a + Kd_a*(erro_angular-memoria_azul_angular.at(0));

        VW[id][0] = V;
        VW[id][1] = W;
        atualiza_memoria_azul(erro_linear,erro_angular);
    }else{
        for (int i = 0; i < (int)memoria_amarelo_linear.size(); i++)
        {
            temp_integral_l = temp_integral_l + memoria_amarelo_linear.at(i);
        }

        for (int i = 0; i < (int)memoria_amarelo_angular.size(); i++)
        {
            temp_integral_a = temp_integral_a + memoria_amarelo_angular.at(i);
        }

        V = Kp_l*erro_linear + Ki_l*temp_integral_l + Kd_l*(erro_linear-memoria_amarelo_linear.at(0));

        W = Kp_a*erro_angular + Ki_a*temp_integral_a + Kd_a*(erro_angular-memoria_amarelo_angular.at(0));

        VW[id][0] = V;
        VW[id][1] = W;
        atualiza_memoria_amarelo(erro_linear,erro_angular);
    }
}

void Strategy::atualiza_memoria_azul(double linear, double angular)
{
    //Removendo últimos elementos
    memoria_azul_linear.pop_back();
    memoria_azul_angular.pop_back();

    //Inserindo novos elementos no começo
    memoria_azul_linear.insert(memoria_azul_linear.begin(),linear);
    memoria_azul_angular.insert(memoria_azul_angular.begin(),angular);
}

void Strategy::atualiza_memoria_amarelo(double linear, double angular)
{
    //Removendo últimos elementos
    memoria_amarelo_linear.pop_back();
    memoria_amarelo_angular.pop_back();

    //Inserindo novos elementos no começo
    memoria_amarelo_linear.insert(memoria_amarelo_linear.begin(),linear);
    memoria_amarelo_angular.insert(memoria_amarelo_angular.begin(),angular);
}


double Strategy::controleAngular(double fi2) // função testada. lembrete : (sinal de w) = -(sinal de fi)
{
    //double W_max = -0.3;    // constante limitante da tangente hiperbólica : deve ser negativa
    double W_max = Wmax;
    double k_ang = 0.2;
    double Waux = 0;
    double fi = fi2/90; // coloca o erro entre -1 e 1

    Waux = W_max*tanh(k_ang*fi); // nгo linear
                              //  W =  kw*fi;                     // proporcional

    Waux = limita_velocidade(Waux, Wmax); //satura em -1 a 1

    return(Waux); //deve retornar um valor entre -1 e 1
}

double Strategy::controleLinear(fira_message::Robot rb,double px, double py)
{
    double  Vaux = 0;
    double  k_lin = 4;   //constante de contração da tangente hiperbólica Rapha colocou 0.8
    double  V_max = Vmax;       //constante limitante da tangente hiperbólica
    double  v_min = 0.5;  	 //módulo da velocidade linear mínima permitida Rapha colocou 0.03
    double  ang_grande = 30; //para ângulos maiores que esse valor o sistema da prioridade ao W, reduzindo o V
    double  dist = distancia(rb, px, py);

    ang_err angulo = olhar(rb, px, py);

    Vaux = V_max*tanh(k_lin*dist*angulo.flag);  //controle não linear de V

    //if (Vaux*angulo.flag < v_min) Vaux = v_min*angulo.flag;  //aplica o valor definido em v_min

    //if (angulo.fi*angulo.flag > ang_grande) V = v_min*angulo.flag;  // controle de prioridade reduzindo V quando "ang_err" for grande
    Vaux = Vaux*abs(cos(angulo.fi*M_PI / 180));// controle de prioridade reduzindo V quando "ang_err" for grande

    Vaux = limita_velocidade(Vaux, Vmax); //satura em -1 a 1

    return(Vaux);
}

ang_err Strategy::olhar(fira_message::Robot rb, double px, double py)   // função testada - ok!
{
      double r = rb.orientation()*180/M_PI; // orientação do robô de -180 a 180
      ang_err angulo;

      angulo.flag = 1;
      angulo.fi = atan2((py - rb.y()) , (px - rb.x()))*180/M_PI;  //ângulo entre -180 e 180

      if (r < 0)  {r = r + 360;}          //muda para 0 a 360
      if (angulo.fi < 0)   {angulo.fi = angulo.fi + 360;}      //muda para 0 a 360

      angulo.fi = angulo.fi - r;  //erro de orientação a ser corrigido

      if (angulo.fi > 180) {angulo.fi = angulo.fi - 360;}  // limita entre -180 e 180
      if (angulo.fi < -180) {angulo.fi = angulo.fi + 360;} // limita entre -180 e 180

      if (angulo.fi > 90) // se for mais fácil, olhar com o fundo...
      {
          angulo.fi = angulo.fi - 180;
          angulo.flag = -1;
      }
      if (angulo.fi < -90) // se for mais fácil, olhar com o fundo...
      {
          angulo.fi = 180 + angulo.fi;
          angulo.flag = -1;
      }

      // angulo.fi é o ângulo a ser corrigido na convenção do sistema de visão. valores entre -90 e 90.
      // angulo.flag é o sinal a ser aplicado na velocidade linear
      return(angulo);
}

double Strategy::distancia(fira_message::Robot rb, double px, double py)
{
      double dist = sqrt( pow((rb.x()-px),2) + pow((rb.y()-py),2) );
      return(dist);
}

double Strategy::limita_velocidade(double valor, double sat)
{
      if (valor > sat) {valor = sat;}
      if (valor < -sat) {valor = -sat;}
      return(valor);
}

Strategy::~Strategy()
{

}

//Alterações Petersson
//Função de saturação dos valores a serem enviados aos vaipara
void Strategy::saturacao(double V[]){
    //limites de x e y
    double lim_x = 0.68;
    double lim_y = 0.58;

    //Satura as posições enviadas
    if (V[0] > lim_x)
        V[0] = lim_x;

    if (V[0] < -lim_x)
        V[0] = -lim_x;

    if (V[1] > lim_y)
        V[1] = lim_y;

    if (V[1] < -lim_y)
        V[1] = -lim_y;
}

//Atualiza a estrutura de dados contendo as posições de todos os robôs
void Strategy::atualiza_pos(fira_message::Robot b0,fira_message::Robot b1,fira_message::Robot b2,fira_message::Robot y0,fira_message::Robot y1,fira_message::Robot y2){
    Strategy::pos_robos[0][0] = b0.x();
    Strategy::pos_robos[0][1] = b0.y();
    Strategy::pos_robos[1][0] = b1.x();
    Strategy::pos_robos[1][1] = b1.y();
    Strategy::pos_robos[2][0] = b2.x();
    Strategy::pos_robos[2][1] = b2.y();

    Strategy::pos_robos[3][0] = y0.x();
    Strategy::pos_robos[3][1] = y0.y();
    Strategy::pos_robos[4][0] = y1.x();
    Strategy::pos_robos[4][1] = y1.y();
    Strategy::pos_robos[5][0] = y2.x();
    Strategy::pos_robos[5][1] = y2.y();

}


//Campo repulsivo linear
void Strategy::calc_repulsao(fira_message::Robot rb, double F[]){
     double raio_adversario = 0.2;  //raio de detecção do adversario

     F[0] = 0;
     F[1] = 0;

     double dist_adversario;

     for(int i = 0 ; i < 5 ; i++){

         dist_adversario = sqrt(pow((rb.x() - pos_robos[i][0]),2) + pow((rb.y() - pos_robos[i][1]),2));

         if (dist_adversario <= raio_adversario && dist_adversario > 0.01){
             double dist_x = rb.x() - pos_robos[i][0];
             double dist_y = rb.y() - pos_robos[i][1];

             F[0] += raio_adversario*dist_x/dist_adversario - dist_x;
             F[1] += raio_adversario*dist_y/dist_adversario - dist_y;
         }

     }
}

//limita o valor de um vetor
void Strategy::converte_vetor(double V[],double raio){

    double dist = sqrt(pow(V[0],2) + pow(V[1],2));

    if (dist > raio){
        V[0] = raio*V[0]/dist;
        V[1] = raio*V[1]/dist;
    }

}

double Strategy::filtro(double V,int id){

    //filtro IIR
    double k = 0.9;

    V = velocidades[id]*(1-k) + V*k;
    velocidades[id] = V;

    return V;
}

void Strategy::goleiro_petersson(fira_message::Robot rb,fira_message::Ball ball, int id){

    double top_limit = 0.17; //largura do gol/2
    double x_desejado = -0.7*lado;

    double dist = distancia(rb,ball.x(),ball.y());
    vector <double> new_pos = {0,0};

    //se a bola estiver longe utiliza o preditor
    if ( dist > 0.3){
        new_pos = {predictedBall.x,predictedBall.y};
    }else{
        new_pos = {ball.x(),ball.y()};
    }

    if(distancia(rb,x_desejado,rb.y()) >= 0.02){ //se o robô está dentro do retângulo

        if(distancia(rb,x_desejado,rb.y()) >= 0.3){
              vaiPara_desviando(rb,x_desejado,0.0,id);
        }else{
              vaiPara(rb,x_desejado,0.0,id);
        }

    }else{

        ang_err angulo = olhar(rb,rb.x(),top_limit + 5); // calcula diferença entre angulo atual e angulo desejado
        if(angulo.fi >= 0.5 || angulo.fi<= -0.5){ //se o robô não está aproximadamente 90 graus
            andarFrente(0,id);
            VW[id][1] = controleAngular(angulo.fi);
        }

        else if(rb.y() < top_limit && rb.y() < new_pos[1]){ //robô abaixo da bola

            if(angulo.flag == 1){
                andarFrente(125,id);
            }
            else{
                andarFundo(125,id);
            }
        }
        else if(rb.y() > -top_limit && rb.y() > new_pos[1]){ //robô acima da bola
            if(angulo.flag == 1){
                andarFundo(125,id);
            }
            else{
                andarFrente(125,id);
            }
        }
        else{
            andarFrente(0,id);
        }
        //gira se a bola estiver muito perto
        if (distancia(rb,ball.x(),ball.y()) < 0.08){
            if((ball.y() < 0 && lado == 1)){
               girarHorario(125,id);
            }
            if((ball.y() > 0 && lado == -1)){
               girarHorario(125,id);
            }
            if((ball.y() > 0 && lado == 1)){
               girarAntihorario(125,id);
            }
            if((ball.y() < 0 && lado == -1)){
               girarAntihorario(125,id);
            }
        }
    }

}

void Strategy::vaiPara_desviando(fira_message::Robot rb,double px,double py,int id){

    double V[2] = {px - rb.x(),py - rb.y()};

    double F[2] = {0,0};

    calc_repulsao(rb,F);

    double ka = 1;
    double kr = 1;

    converte_vetor(V,0.1);
    converte_vetor(F,0.2);

    double new_pos[2] = {rb.x() + ka*V[0] + kr*F[0],rb.y() + ka*V[1] + kr*F[1]};

    Strategy::saturacao(new_pos);

    vaiPara(rb,new_pos[0],new_pos[1],id);

    filtro(VW[id][0],id);
}


vector<double> Strategy::inserirRRT(vector<double> V_in,vector<double> V_out,int opcao){
    //Se a opção for zero concatena os vetores, senao apaga tudo e insere o novo vetor no lugar
    if (opcao == 0){
        V_out.insert(V_out.end(),V_in.begin(),V_in.end());
    }else{
        V_out.clear();
        V_out.insert(V_out.end(),V_in.begin(),V_in.end());
    }
    return V_out;
}

void Strategy::zagueiro2(fira_message::Robot rb, double xbola, double ybola, int id){
    double x_penalti =  0.4*lado; //bolinha do penalti
    double x_meio_de_campo = 0.0; //centro do campo
    double x_radius = 0.2*lado; //raio do circulo do meio de campo
    double y_top = 0.65; // metade da largura do campo
    double ala_deepth = 0.35; //metade da área do goleiro
    double K_press = 0.3*lado; //ṕonto entre meio de campo e marca do penalti
    double ajuste = 0.15; //ajuste de lugar onde o zagueiro, quando a bola está proximo do escanteio do seu time
    double ajuste2 = 0.1; //ajuste de distância que o zagueiro deve estar para ir na bola no campo de ataque
    int teste = 0; //armazena se entrou no else

    if(lado == 1){
        //lado azul
        if(xbola > x_penalti)
        {    //Se a Bola estiver na zona "A"
            vaiPara_desviando(rb,x_meio_de_campo + K_press,ybola,id);
            if((distancia(rb,xbola,ybola)< ajuste2) && rb.x() < xbola){
                //Se o robo estiver perto da bola e atrás dela
                vaiPara(rb,predictedBall.x,predictedBall.y,id);
            }
        }
        else if(xbola >= x_meio_de_campo && (ybola < (y_top - ala_deepth) && ybola > (ala_deepth - y_top)))
        {    //Se a Bola estiver na zona "B_mid"
            vaiPara_desviando(rb,-x_penalti,ybola,id);
            if((distancia(rb,xbola,ybola)< ajuste2) && rb.x() < xbola){
                //Se o robo estiver perto da bola e atrás dela
                vaiPara(rb,predictedBall.x,predictedBall.y,id);
            }
        }
        else if((xbola >= x_meio_de_campo) && (ybola > (y_top - ala_deepth) || ybola < (ala_deepth - y_top)))
        {    //Se a Bola estiver na zona "B_top" ou "B_bot"
            vaiPara_desviando(rb,-x_radius,ybola,id);
        }
        else if(xbola < x_meio_de_campo && xbola > rb.x() && (ybola < (y_top - ala_deepth) && ybola > (ala_deepth - y_top)))
        {    //Se a Bola estiver na zona "C"
            vaiPara_desviando(rb,xbola,ybola,id);
        }
        else if((xbola < x_meio_de_campo && ybola > (y_top - ala_deepth +ajuste)))
        {   //Se a Bola estiver na zona "D"
            vaiPara_desviando(rb,xbola,y_top - ala_deepth + ajuste,id);
        }
        else if((xbola < x_meio_de_campo) && ybola < (ala_deepth - y_top  -ajuste))
        {    //Se a Bola estiver na zona "E"
            vaiPara_desviando(rb,xbola,ala_deepth - y_top - ajuste,id);
        }
        else // se não estiver em nenhuma das regiôes
        {
            vaiPara_desviando2(rb,-x_penalti -0.1, 0.0,id);
            teste = 1; //entrou aqui
            /*if(sqrt( pow((-x_penalti -0.1 -predictedBall.x),2) + pow((0.0 - predictedBall.y),2) ) < 0.2){
                vaiPara_desviando(rb,-x_penalti +0.1, 0.0,id);
            }*/
        }
        //gira se a bola estiver muito perto
        if ((distancia(rb,xbola,ybola) < 0.08) && (((rb.x() < xbola)&&teste == 0))){
            if(ybola < 0){
               girarHorario(125,id);
            }
            if(ybola > 0){
               girarAntihorario(125,id);
            }
        }
     }else{
        //lado amarelo
        if(xbola < x_penalti)
        {    //Se a Bola estiver na zona "A"
            vaiPara_desviando(rb,x_meio_de_campo + K_press,ybola,id);
            if((distancia(rb,xbola,ybola)< ajuste2) && rb.x() > xbola){
                //Se o robo estiver perto da bola e atrás dela
                vaiPara(rb,predictedBall.x,predictedBall.y,id);
            }
        }
        else if(xbola <= x_meio_de_campo && (ybola < (y_top - ala_deepth) && ybola > (ala_deepth - y_top)))
        {    //Se a Bola estiver na zona "B_mid"
            vaiPara_desviando(rb,-x_penalti,ybola,id);
            if((distancia(rb,xbola,ybola)< ajuste2) && rb.x() > xbola){
                //Se o robo estiver perto da bola e atrás dela
                vaiPara(rb,predictedBall.x,predictedBall.y,id);
            }
        }
        else if((xbola <= x_meio_de_campo) && (ybola > (y_top - ala_deepth) || ybola < (ala_deepth - y_top)))
        {    //Se a Bola estiver na zona "B_top" ou "B_bot"
            vaiPara_desviando(rb,-x_radius,ybola,id);
        }
        else if(xbola > x_meio_de_campo && xbola < rb.x() && (ybola < (y_top - ala_deepth) && ybola > (ala_deepth - y_top)))
        {    //Se a Bola estiver na zona "C"
            vaiPara_desviando(rb,xbola,ybola,id);
        }
        else if((xbola > x_meio_de_campo && ybola > (y_top - ala_deepth +ajuste)))
        {   //Se a Bola estiver na zona "D"
            vaiPara_desviando(rb,xbola,y_top - ala_deepth + ajuste,id);
        }
        else if((xbola > x_meio_de_campo) && ybola < (ala_deepth - y_top  -ajuste))
        {    //Se a Bola estiver na zona "E"
            vaiPara_desviando(rb,xbola,ala_deepth - y_top - ajuste,id);
        }
        else // se não estiver em nenhuma das regiôes
        {
            vaiPara_desviando2(rb,-x_penalti +0.1, 0.0,id);
            teste = 1;
            /*if(sqrt( pow((-x_penalti +0.1 -predictedBall.x),2) + pow((0.0 - predictedBall.y),2) ) < 0.2){
                vaiPara_desviando(rb,-x_penalti -0.1, 0.0,id);
            }*/
        }
        //gira se a bola estiver muito perto
        if ((distancia(rb,xbola,ybola) < 0.08) && ((rb.x() > xbola)&&teste == 0)){
            if(ybola > 0){
               girarHorario(125,id);
            }
            if(ybola < 0){
               girarAntihorario(125,id);
            }
        }
    }
}


//serve apenas para o zagueiro2
void Strategy::vaiPara_desviando2(fira_message::Robot rb,double px,double py,int id){

    double V[2] = {px - rb.x(),py - rb.y()};

    double F[2] = {0,0};

    calc_repulsao2(rb,F);

    double ka = 1;
    double kr = 1;

    converte_vetor(V,0.1);
    converte_vetor(F,0.2);

    double new_pos[2] = {rb.x() + ka*V[0] + kr*F[0],rb.y() + ka*V[1] + kr*F[1]};

    Strategy::saturacao(new_pos);

    vaiPara(rb,new_pos[0],new_pos[1],id);

    filtro(VW[id][0],id);
}

//Campo repulsivo linear
//serve apenas para o zagueiro2
void Strategy::calc_repulsao2(fira_message::Robot rb, double F[]){
     double raio_adversario = 0.2;  //raio de detecção do adversario

     F[0] = 0;
     F[1] = 0;

     double dist_adversario;

     for(int i = 0 ; i < 5 ; i++){

         dist_adversario = sqrt(pow((rb.x() - pos_robos[i][0]),2) + pow((rb.y() - pos_robos[i][1]),2));

         if (dist_adversario <= raio_adversario && dist_adversario > 0.01){
             double dist_x = rb.x() - pos_robos[i][0];
             double dist_y = rb.y() - pos_robos[i][1];

             F[0] += raio_adversario*dist_x/dist_adversario - dist_x;
             F[1] += raio_adversario*dist_y/dist_adversario - dist_y;
         }

     }
     dist_adversario = sqrt(pow((rb.x() - predictedBall.x),2) + pow((rb.y() - predictedBall.y),2));
     if (dist_adversario <= raio_adversario){
         double dist_x = rb.x() - predictedBall.x;
         double dist_y = rb.y() - predictedBall.y;

         F[0] += raio_adversario*dist_x/dist_adversario - dist_x;
         F[1] += raio_adversario*dist_y/dist_adversario - dist_y;
     }
}

//Atacante de Lázaro e cone
void Strategy::atacante_todos(Team rb,Team adversario, fira_message::Ball ball, int id, int idzag)
{

    double alpha = M_PI/8;
    double meioGolx = -0.75*lado;
    double beta = atan2(rb[id].x() - meioGolx , rb[id].y());
    int K = round(beta/alpha);
    double gain;
  //double resultante_2 = {0,0};

    if(K<2)
        gain = 0.4;
    else if(K>=2 && K <= 3)
        gain = 0.35;
    else if(K>3 && K < 5)
        gain = 0.4;
    else if(K>=5 && K < 7)
        gain = 0.35;
    else
        gain = 0.4;

    delete resultante_2;
    resultante_2 = new vector<double>();

    //Determinação da origem do vetor resultante
    resultante_2->push_back(rb[id].x());
    resultante_2->push_back(rb[id].y());

    //distância atacante-bola
    double dist =  sqrt(pow(ball.x()-rb[id].x(),2.0)+pow( ball.y()-rb[id].y(),2.0));
    //Distancia do zagueiro até a bola
    double zag_dist = sqrt(pow(ball.x()-rb[idzag].x(),2.0)+pow( ball.y()-rb[idzag].y(),2.0));

    double componenteX;
    double componenteY;

    double k = dist;
    if(dist < 0.2)
        k  = 0.2;

    (*resultante_2)[0]+=-k*sin(atan2((rb[id].x() - ball.x()),(rb[id].y() - ball.y())));
    (*resultante_2)[1]+=-k*cos(atan2((rb[id].x() - ball.x()),(rb[id].y() - ball.y())));


    if(lado > 0){//azul


        if(ball.x() < rb[id].x())
        {
            int a = 0;
            //Consideração do vetor de predição
            //if(abs(ball.y()-rb[id].y()) > 0.3)
                //a = 1;

            int c = 0;
            if((ball.x()/rb[id].x()<1 && ball.x()/rb[id].x()>0.8))
                c = 1;

            double repulsivoX;
            double repulsivoY;

            if(ball.y() > rb[id].y())
            {
                repulsivoX = (c*0.1+0.1)*sin(0+(c*0.5*M_PI));
                repulsivoY = (c*0.1+0.1)*cos(0+(c*0.5*M_PI));
            }else
            {
                repulsivoX = (c*0.1+0.1)*sin(M_PI-(c*0.5*M_PI));
                repulsivoY = (c*0.1+0.1)*cos(M_PI-(c*0.5*M_PI));
            }

            //Componentes do Vetor direção
            double angle = atan2(ball.x()-predictedBall.x,ball.y()-predictedBall.y);
            componenteX = /*-c*ball.x()*/ -  a*0.1*sin(angle) - repulsivoX;
            componenteY = /*-c*ball.y()*/ -  a*0.1*cos(angle) - repulsivoY;

            (*resultante_2)[0]+= componenteX;
            (*resultante_2)[1]+= componenteY;

        }else
        {

            if(dist < 0.08)
            {
                (*resultante_2)[0]+=-gain*sin(beta+M_PI);
                (*resultante_2)[1]+=-gain*cos(beta+M_PI);
            }


            if((ball.x() < 0.6 && ball.y() < -0.2)&& (rb[id].y() > ball.y())/*&&(abs(ball.y()-blue[id].x())>0.07)*/)
            {
                //Vetor de corrção
                componenteX = /*ball.x()*/ -  0.1*sin(atan2(ball.x() - meioGolx,ball.y() - 0.0)/*+M_PI*/);
                componenteY = /*ball.y()*/ -  0.1*cos(atan2(ball.x() - meioGolx,ball.y() - 0.0)/*+M_PI*/);

                (*resultante_2)[0]+=componenteX;
                (*resultante_2)[1]+=componenteY;
            }else if((ball.x() < 0.6 && ball.y() > 0.2)&&(rb[id].y() < ball.y())/*&&(abs(ball.y()-blue[id].x())>0.07)*/)
            {
                //Vetor de corrção
                componenteX = /*ball.x()*/ -  0.1*sin(atan2(ball.x() - meioGolx,ball.y() - 0.0)/*+M_PI*/);
                componenteY = /*ball.y()*/ -  0.1*cos(atan2(ball.x() - meioGolx,ball.y() - 0.0)/*+M_PI*/);

                (*resultante_2)[0]+=componenteX;
                (*resultante_2)[1]+=componenteY;

            }else if(dist > 0.2)
            {
                (*resultante_2)[0]+= -0.1*sin(atan2(ball.x() - meioGolx,ball.y() - 0.0));
                (*resultante_2)[1]+= -0.1*cos(atan2(ball.x() - meioGolx,ball.y() - 0.0));
            }

        }


        if(ball.x()==0)
        {
            vaiPara(rb[id],ball.x(),ball.y(),id);
        }
        else
        {
            if(ball.x()<=0)
            {

                if(zag_dist/dist < 0.9)
                {
                    //Comportamento sem bola : Defensivo

                    /* dist_target = distância do alvo para a bola;
                     * target = alvo da marcação;
                     * (zag_dist/dist) = razão entre as distâcias do zagueiro para
                     o atacante;
                     */

                    //Calculo do alvo de marcação
                    double dist_target = 0.0;
                    int target = 0;
                    for(int player = 0; player < 3; player++)
                    {
                        double dist_adv = sqrt(pow(ball.x()-adversario[player].x(),2.0)+pow( ball.y()-adversario[player].y(),2.0));
                        if(!(adversario[player].x() > 0.65 || adversario[player].x() < -0.65) && (dist_adv > dist_target))
                        {
                            dist_target = dist_adv;
                            target = player;
                        }
                    }

                    //Pos com relação ball-alvo
                    double attachX = ball.x() - (0.8*dist_target)*sin(atan2(ball.x() - adversario[target].x(),ball.y() - adversario[target].y()));
                    double attachY = ball.y() - (0.8*dist_target)*cos(atan2(ball.x() - adversario[target].x(),ball.y() - adversario[target].y()));

                    vaiPara_desviando(rb[id],attachX,attachY,id);
                }
                else
                    vaiPara_desviando(rb[id],0,(*resultante_2)[1]-0.1,id);

            }else
            {
                //Comportamento Ofensivo - Posicionamento

               //Cálculo da posição
                double attachX =  ball.x() + 0.45*sin(atan2(ball.x() - rb[idzag].x(),ball.y() - rb[idzag].y()));
                //double attachY =  ball.y() + 0.45*cos(atan2(ball.x() - rb[1].x(),ball.y() - rb[1].y()));
                int c;
                if(rb[idzag].y()>0)
                    c = -1;
                else
                    c = 1;

                if(zag_dist/dist < 0.3)
                    vaiPara_desviando(rb[id],attachX-0.1,c*0.25,id);
                else
                    vaiPara_desviando(rb[id],(*resultante_2)[0],(*resultante_2)[1],id);

                /*double distToGoal = sqrt(pow(rb[id].x()-meioGolx,2.0)+pow(rb[id].y()-0,2.0));
                if(distToGoal < 0.5 && ball.x()>rb[id].x() && (ball.y() < 0.20 && ball.y()>-0.20) )
                    vaiPara(rb[id],ball.x(),ball.y(),id);*/


            }
        }

    }else{ //Amarelo

        if((ball.x() > rb[id].x()))
        {
            int a = 0;
            //Consideração do vetor de predição
            //if(abs(ball.y()-rb[id].y()) > 0.3)
                //a = 1;

            int c = 0;
            if((ball.x()/rb[id].x()<1.1 && ball.x()/rb[id].x()>1))
                c = 1;

            double repulsivoX;
            double repulsivoY;

            if(ball.y() > rb[id].y())
            {
                repulsivoX = (c*0.1+0.1)*sin(0+(c*0.5*M_PI));
                repulsivoY = (c*0.1+0.1)*cos(0+(c*0.5*M_PI));
            }else
            {
                repulsivoX = (c*0.1+0.1)*sin(M_PI-(c*0.5*M_PI));
                repulsivoY = (c*0.1+0.1)*cos(M_PI-(c*0.5*M_PI));
            }

            //Componentes do Vetor direção
            double angle = atan2(ball.x()-predictedBall.x,ball.y()-predictedBall.y);
            componenteX = /*-c*ball.x()*/ -  a*0.1*sin(angle) - repulsivoX;
            componenteY = /*-c*ball.y()*/ -  a*0.1*cos(angle) - repulsivoY;

            (*resultante_2)[0]+= componenteX;
            (*resultante_2)[1]+= componenteY;

        }else
        {
            if(dist < 0.08)
            {
                (*resultante_2)[0]+=-gain*sin(beta+M_PI);
                (*resultante_2)[1]+=-gain*cos(beta+M_PI);
            }

            if((ball.x() > -0.6 && ball.y() < -0.2)&& (rb[id].y() > ball.y())/*&&(abs(ball.y()-blue[id].x())>0.07)*/)
            {
                //Vetor de corrção
                componenteX = /*ball.x()*/ -  0.1*sin(atan2(ball.x() - meioGolx,ball.y() - 0.0)/*+M_PI*/);
                componenteY = /*ball.y()*/ -  0.1*cos(atan2(ball.x() - meioGolx,ball.y() - 0.0)/*+M_PI*/);

                (*resultante_2)[0]+=componenteX;
                (*resultante_2)[1]+=componenteY;
            }else if((ball.x() > -0.6 && ball.y() > 0.2)&&(rb[id].y() < ball.y())/*&&(abs(ball.y()-blue[id].x())>0.07)*/)
            {
                //Vetor de corrção
                componenteX = /*ball.x()*/ -  0.1*sin(atan2(ball.x() - meioGolx,ball.y() - 0.0)/*+M_PI*/);
                componenteY = /*ball.y()*/ -  0.1*cos(atan2(ball.x() - meioGolx,ball.y() - 0.0)/*+M_PI*/);

                (*resultante_2)[0]+=componenteX;
                (*resultante_2)[1]+=componenteY;

            }else if(dist > 0.2)
            {
                (*resultante_2)[0]+= -0.1*sin(atan2(ball.x() - meioGolx,ball.y() - 0.0));
                (*resultante_2)[1]+= -0.1*cos(atan2(ball.x() - meioGolx,ball.y() - 0.0));
            }

        }


        if(ball.x()==0)
        {
            vaiPara(rb[id],ball.x(),ball.y(),id);
        }
        else
        {
            if(ball.x()>=0)
            {

                if(zag_dist/dist < 0.9)
                {
                    //Comportamento sem bola : Defensivo

                    /* dist_target = distância do alvo para a bola;
                     * target = alvo da marcação;
                     * (zag_dist/dist) = razão entre as distâcias do zagueiro para
                     o atacante;
                     */

                    //Calculo do alvo de marcação
                    double dist_target = 0.0;
                    int target = 0;
                    for(int player = 0; player < 3; player++)
                    {
                        double dist_adv = sqrt(pow(ball.x()-adversario[player].x(),2.0)+pow( ball.y()-adversario[player].y(),2.0));
                        if(!(adversario[player].x() > 0.65 || adversario[player].x() < -0.65) && (dist_adv > dist_target))
                        {
                            dist_target = dist_adv;
                            target = player;
                        }
                    }

                    //Pos com relação ball-alvo
                    double attachX = ball.x() - (0.8*dist_target)*sin(atan2(ball.x() - adversario[target].x(),ball.y() - adversario[target].y()));
                    double attachY = ball.y() - (0.8*dist_target)*cos(atan2(ball.x() - adversario[target].x(),ball.y() - adversario[target].y()));

                    vaiPara_desviando(rb[id],attachX,attachY,id);
                }
                else
                    vaiPara_desviando(rb[id],0,(*resultante_2)[1],id);

            }else
            {
                //Comportamento Ofensivo - Posicionamento

               //Cálculo da posição
                double attachX =  ball.x() + 0.45*sin(atan2(ball.x() - rb[idzag].x(),ball.y() - rb[idzag].y()));
                //double attachY =  ball.y() + 0.45*cos(atan2(ball.x() - rb[1].x(),ball.y() - rb[1].y()));
                int c;
                if(rb[idzag].y()>0)
                    c = -1;
                else
                    c = 1;

                if(zag_dist/dist < 0.3)
                    vaiPara_desviando(rb[id],attachX-0.1,c*0.25,id);
                else
                    vaiPara_desviando(rb[id],(*resultante_2)[0],(*resultante_2)[1],id);

                /*double distToGoal = sqrt(pow(rb[id].x()-meioGolx,2.0)+pow(rb[id].y()-0,2.0));
                if(distToGoal < 0.5 && ball.x()>rb[id].x() && (ball.y() < 0.20 && ball.y()>-0.20) )
                    vaiPara(rb[id],ball.x(),ball.y(),id);*/


            }

        }
    }
}


void Strategy::goleiro_petersson2(fira_message::Robot rb,fira_message::Ball ball, int id){

    double top_limit = 0.17; //largura do gol/2
    double x_desejado = -0.7*lado;

    double dist = distancia(rb,ball.x(),ball.y());
    vector <double> new_pos = {0,0};

    //se a bola estiver longe do goleiro utiliza o preditor para ajeitar sua posição
    if ( dist > 0.3){
        new_pos = {predictedBall.x,predictedBall.y};
    }else{
        new_pos = {ball.x(),ball.y()};
    }

    if(dist < 0.12 && ball.x() > rb.x() && rb.x() < -0.6 && ball.y() <= top_limit && ball.y() >=-top_limit && lado == 1){
        vaiPara(rb,ball.x(),ball.y(),id);
    }else if(dist < 0.12 && ball.x() < rb.x() && rb.x() > 0.6 && ball.y() <= top_limit && ball.y() >=-top_limit && lado == -1){
        vaiPara(rb,ball.x(),ball.y(),id);
    }else{
        //Verifica se o robô está perto do centro do gol
        if(distancia(rb,x_desejado,rb.y()) >= 0.02){

            if(distancia(rb,x_desejado,rb.y()) >= 0.3){
                  vaiPara_desviando(rb,x_desejado,0.0,id);
            }else{
                  vaiPara(rb,x_desejado,0.0,id);
            }

        }else{

            ang_err angulo = olhar(rb,rb.x(),top_limit + 5); // calcula diferença entre angulo atual e angulo desejado
            if(angulo.fi >= 0.5 || angulo.fi<= -0.5){ //se o robô não está aproximadamente 90 graus
                andarFrente(0,id);
                VW[id][1] = controleAngular(angulo.fi);
            }

            else if(rb.y() < top_limit && rb.y() < new_pos[1]){ //robô abaixo da bola

                if(angulo.flag == 1){
                    andarFrente(125,id);
                }
                else{
                    andarFundo(125,id);
                }
            }
            else if(rb.y() > -top_limit && rb.y() > new_pos[1]){ //robô acima da bola
                if(angulo.flag == 1){
                    andarFundo(125,id);
                }
                else{
                    andarFrente(125,id);
                }
            }
            else{
                andarFrente(0,id);
            }
            //gira se a bola estiver muito perto do goleiro
            if (distancia(rb,ball.x(),ball.y()) < 0.08){
                if((ball.y() < rb.y() && lado == 1)){
                   girarHorario(125,id);
                }
                if((ball.y() > rb.y() && lado == -1)){
                   girarHorario(125,id);
                }
                if((ball.y() > rb.y() && lado == 1)){
                   girarAntihorario(125,id);
                }
                if((ball.y() < rb.y() && lado == -1)){
                   girarAntihorario(125,id);
                }
            }
        }
    }

}
