//author  Renato Sousa, 2018
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
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/mat.hpp>
#include <opencv4/opencv2/calib3d/calib3d.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

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




    // generate space
    cv::Mat image;
    cv::Vec3b val;

    image= cv::imread("campo.png",cv::IMREAD_COLOR);
    if(!image.data)
      std::cout << "nao abriu campo.png" << std::endl;

    cv::namedWindow("Plot Virtual", cv::WINDOW_AUTOSIZE);



    RoboCupSSLClient client;
    client.open(false);
    fira_message::sim_to_ref::Environment packet;

    GrSim_Client grSim_client;

    //inicialização da classe estratégia
    Strategy estrategia;
    Team *blue = NULL;
    Team *yellow = NULL;


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
                estrategia.predict_ball(ball);
                //printf("-Ball:  POS=<%9.2f,%9.2f> \n",ball.x(),ball.y());

                //printf("-[Geometry Data]-------\n");
                const fira_message::Field & field = packet.field();
                //printf("Field Dimensions:\n");
                //printf("  -field_length=%f (mm)\n",field.length());
                //printf("  -field_width=%f (mm)\n",field.width());
                //printf("  -goal_width=%f (mm)\n",field.goal_width());
                //printf("  -goal_depth=%f (mm)\n",field.goal_depth());

                delete blue;
                delete yellow;
                //Blue
                blue = new Team(detection.robots_blue(0),detection.robots_blue(1),detection.robots_blue(2));
               //Yellow
                yellow = new Team(detection.robots_yellow(0),detection.robots_yellow(1),detection.robots_yellow(2));

                estrategia.strategy_blue(*blue,*yellow,ball,field);


                //Enviando velocidades
                for(int i = 0;i < estrategia.qtdRobos;i++)
                    grSim_client.sendCommand(estrategia.vRL[i][1],estrategia.vRL[i][0],i);

                //Debug
                //printf("V:%f\n",sqrt(pow(b2.vx(),2)+pow(b2.vy(),2)));
                //printf("W:%f\n",b2.vorientation());
                //printf("Venviado:%f\n",estrategia.VW[2][0]);
                //printf("Wenviado:%f\n",estrategia.VW[2][1]);
                //printf("VR:%f\n",estrategia.vRL[2][0]);
                //printf("VL:%f\n",estrategia.vRL[2][1]);

                //Plot Virtual
                image= cv::imread("campo.png",cv::IMREAD_COLOR);
                //Bola
                circle(image,Point((ball.x()+0.75)*400,(-ball.y()+0.65)*400),5,Scalar(0,110,255),7);
                //Time blue
                circle(image,Point(((*blue)[0].x()+0.75)*400,(-(*blue)[0].y()+0.65)*400),8,Scalar(255,0,0),12);
                circle(image,Point(((*blue)[1].x()+0.75)*400,(-(*blue)[1].y()+0.65)*400),8,Scalar(255,0,0),12);
                circle(image,Point(((*blue)[2].x()+0.75)*400,(-(*blue)[2].y()+0.65)*400),8,Scalar(255,0,0),12);
                //Time Amarelo
                circle(image,Point(((*yellow)[0].x()+0.75)*400,(-(*yellow)[0].y()+0.65)*400),8,Scalar(0,255,255),12);
                circle(image,Point(((*yellow)[1].x()+0.75)*400,(-(*yellow)[1].y()+0.65)*400),8,Scalar(0,255,255),12);
                circle(image,Point(((*yellow)[2].x()+0.75)*400,(-(*yellow)[2].y()+0.65)*400),8,Scalar(0,255,255),12);

                //RRT plot
                for(int node = 0; node < estrategia.rrt->GetNumNodes(); node++)
                {
                    double x = estrategia.rrt->GetNodeState(node).x;
                    double y = estrategia.rrt->GetNodeState(node).y;

                    circle(image,Point((x+0.75)*400,(-y+0.65)*400),2,Scalar(0,255,255),1);

                    list<int>::iterator it = estrategia.rrt->GetNodeNeighbors(node).begin();

                    for(int i = 0; i < (int)estrategia.rrt->GetNodeNeighbors(node).size();i++)
                    {
                        double _x = estrategia.rrt->GetNodeState((*it) + i).x;
                        double _y = estrategia.rrt->GetNodeState((*it) + i).y;
                        line(image,Point((x+0.75)*400,(-y+0.65)*400),Point((_x+0.75)*400,(-_y+0.65)*400),Scalar(0,255,255),1);
                    }

                }
                circle(image,Point(((*blue)[0].x()+0.75)*400,(-(*blue)[0].y()+0.65)*400),0.4*400,Scalar(0,255,0),4);
                vector<State> metas = estrategia.rrt->GetOptimaNodes();
                for(int p = 0; p < (int)metas.size(); p++)
                {
                    double x = metas[p].x;
                    double y = metas[p].y;

                    circle(image,Point((x+0.75)*400,(-y+0.65)*400),4,Scalar(0,0,255),5);

                }


                imshow( "Plot Virtual", image );
                waitKey(1);




            }
        }
    }

    destroyAllWindows();


    return 0;
}
