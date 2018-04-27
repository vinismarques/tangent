#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <cfloat>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <tf/transform_datatypes.h>

// #define ARRAY_SIZE(array) (sizeof((array))/sizeof((array[0])))

//Variaveis Globais Para Leitura de Dados
nav_msgs::Odometry current_pose;
sensor_msgs::LaserScan current_laser;

//Funcao Callback do Laser
void lasercallback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
    current_laser = *laser;
    // printf("%lu\n", sizeof(current_laser.ranges));
    // printf("%lu\n", ARRAY_SIZE(current_laser.ranges));
    // int array_teste[10] = {1, 2, 3, 4, 5};
    // printf("%lu\n", ARRAY_SIZE(array_teste));

    // for (int i = 1; i <= ARRAY_SIZE(current_laser.ranges); i++)
    // {
    //     printf("%lf ", current_laser.ranges[i]);
    // }
    // printf("\n");ARRAY_SIZE(current_laser.ranges)
    // std::cout << "Teste\n";
    // printf("%lu\n", ARRAY_SIZE(current_laser.ranges));

    return;
}

//Funcao Callback da Pose
void posecallback(const nav_msgs::Odometry::ConstPtr& pose)
{
    current_pose = *pose;
	
    return;
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "Path");  // Inicializacao do Nodo

    ros::NodeHandle n; 		     

    // Configuracao do topico a ser publicado
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10); 

    // Configuracao dos topicos a serem lidos
    ros::Subscriber sub = n.subscribe("/base_scan", 10, lasercallback);	
    ros::Subscriber sub1 = n.subscribe("/base_pose_ground_truth", 10, posecallback);
   
    // Define a frequencia do no
    ros::Rate loop_rate(100);

    // Declaracoes
    geometry_msgs::Twist speed_create;  // Comando de Velocidade
    double v1=0.0, v2=0.0;              // Velocidades
    double goalx, goaly;                 // Alvos
    double orientation;					// Orientacao do robo
    double x_robo;
    double y_robo;
    double alfa = 0, beta = 0, d = 0, h = 0; //d = distancia, h = altura

    // printf("argc=%d\n", argc);

    // Faz Leitura dos Parametros para o GOAL
    if(argc == 1){
        printf("Definido Padrao Aleatorio\n");
        srand(time(NULL));
        goalx=double(rand()%200)/10.0;
        srand(time(NULL));
        goaly=double(rand()%150)/10.0;
        
        // Printa o GOAL no terminal
        ROS_INFO("GOAL aleatorio: x=%lf, y=%lf", goalx, goaly);
    }else{
        if(argc == 3){
            goalx=atof(argv[1]);
            goaly=atof(argv[2]);
            printf("Definido GOAL %lf %lf\n", goalx, goaly);
        }else{
            printf("Error in Goal\n");
            return -1;
        }
    }

    //Loop Principal
    while(ros::ok()) 
    {  
                       
        // Pega a orientacao em graus (-180,180)
        orientation = tf::getYaw(current_pose.pose.pose.orientation);// getYaw recebe quaternion e converte para radianos
 	x_robo = current_pose.pose.pose.position.x;
	y_robo = current_pose.pose.pose.position.y;
        beta = orientation*180/M_PI;// Converte de radianos para graus
	d = sqrt((goalx - x_robo)*(goalx - x_robo) + (goaly - y_robo)*(goaly - y_robo));
	h = goaly - y_robo;
	alfa = acos(h/d)*(180/M_PI);
	
	//definicoes de quadrante
	//segundo quadrante	
	if(h > 0 && goalx - x_robo < 0 ){
		alfa = alfa + 90;
	}

	//terceiro quadrante
	if(h < 0 && goalx - x_robo < 0){
		alfa = alfa - 270;
	}

	//quarto quadrante	
	if(h < 0 && goalx - x_robo > 0 ){
		alfa = alfa - 180;
	}

	if(beta - alfa > 0){
		v2=-0.1;	
	}
	
	if(beta - alfa < 0){
		v2=0.1;	
	}

	if(beta - alfa == 0){
		v2=0;	
	}
	
	ROS_INFO("%lg, %lg, %lg, %lg", beta, alfa, d, h); // Debugging

        // CONTINUAR LOGICA
        // criar funcao que recebe goal e retorna velocidades
        // usar essa funcao para passar goals intermediarios
        // usar RRT (usar goal como aleatorio)?	

        // Envia Sinal de Velocidade
        speed_create.linear.x=v1;
        speed_create.angular.z=v2;
        vel_pub.publish(speed_create);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}