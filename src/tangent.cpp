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
// #define k1 2.0
// #define k2 0.06

//Variaveis Globais Para Leitura de Dados
nav_msgs::Odometry current_pose;
sensor_msgs::LaserScan current_laser;

//Funcao Callback do Laser
void lasercallback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
    current_laser = *laser;

    return;
}

//Funcao Callback da Pose
void posecallback(const nav_msgs::Odometry::ConstPtr& pose)
{
    current_pose = *pose;
	
    return;
}

//Funcao Modulo
double modulo(double valor)
{
    return valor>=0.0?valor:-valor;
}

//Funcao que transforma orientacao de radianos para graus (0 a 360)
double graus(double orientacao)
{
    orientacao = orientacao*180/M_PI;// Converte de radianos para graus

    if(orientacao <= -180 || orientacao > 0){

        return modulo(orientacao);
    }
    if(orientacao < 0){

        return orientacao+360;
    }
}

// Funcao que rotaciona de acordo com diferenca de angulos (goal-robo)
double rotaciona(double erro_ang, double prec_ang, double k2){
    if(modulo(erro_ang) > prec_ang){
        // Define sentido de rotacao
        // Escolhe o com menor diferenca
        if(erro_ang > 0 && erro_ang <= 180){    // Rotaciona no sentido anti-horario
            
            return modulo(erro_ang)*k2;     
        }
        else{                                   // Rotaciona no sentido horario
            
            return -modulo(erro_ang)*k2;    
        }
    }else{                                      // Precisao angular atingida, para de rotacionar
        
        return 0.0;                         
    }
}

// Funcao que trata o movimento linear
double anda(double erro_lin, double prec_lin, double k1){
    if(erro_lin >= prec_lin){               // Realimenta o erro multiplicado por um ganho

        return modulo(erro_lin)*k1;
    }else{                                  // Precisao linear atingida, para de andar

        return 0.0;
    }
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
    ros::Rate loop_rate(20);

    // Declaracoes
    geometry_msgs::Twist speed_create;  // Comando de Velocidade
    double v1=0.0, v2=0.0;              // Velocidades
    double goalx, goaly;                 // Alvos
    double orientacao;					// Orientacao do robo
    double x_robo;
    double y_robo;
    double alfa = 0, beta = 0, erro_lin, erro_ang = 0;
    int estado = 1;
    double prec_ang = 0.1, prec_lin = 0.1;
    double x_dif, y_dif;
    double k1 = 2.0, k2 = 0.06;
    
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

    sleep(1); // Aguarda 1 seg antes de iniciar para evitar warnings de "Quaternion Not Properly Normalized"
    
    //Loop Principal
    while(ros::ok()) 
    {  
        orientacao = tf::getYaw(current_pose.pose.pose.orientation);    // getYaw recebe quaternion e converte para radianos
        beta = graus(orientacao);                                       // Converte a orientacao do robo para graus

        x_robo = current_pose.pose.pose.position.x;     // Posicao atual do robo em x
        y_robo = current_pose.pose.pose.position.y;     // Posicao atual do robo em y

        x_dif = goalx - x_robo;                         // Diferenca entre posicao x do goal e do robo
        y_dif = goaly - y_robo;                         // Diferenca entre posicao y do goal e do robo
        
        alfa = atan2(y_dif, x_dif);                     // Angulo alfa pode ser calculado com a tangente
        alfa = graus(alfa);                             // Converte de radianos para graus

        erro_lin = sqrt(x_dif*x_dif + y_dif*y_dif);     // Define o erro linear
        erro_ang = alfa - beta;                         // Define o erro angular
 
        // Estado que rotaciona o robo na direcao do alvo
        if(estado == 1){ 
            if(modulo(erro_ang) < prec_ang ){           // Virado para o goal, vai para o proximo estado
                v2 = 0.0;
                estado = 2;                
            }else{                                      // Continua movimento angular
                v2 = rotaciona(erro_ang, prec_ang, k2);
            }
        }
        
        // Estado que faz o movimento linear
        if(estado == 2){
            if(modulo(erro_lin) <= prec_lin){           // Chegou no goal, vai para o proximo estado
                v1 = 0.0;
                estado = 3;                
            }else if(modulo(erro_ang) > prec_ang){      // Corrige a posicao antes de continuar
                v1 = 0.0;                               
                estado = 1;
            }else{                                      // Continua movimento linear
                v1 = anda(erro_lin, prec_lin, k1);
            }
        }

        // Estado que faz o contorno de obstaculos
        if(estado == 3){
            
        }

        // Estado final, encerra o nodo
        if(estado == 4){
            ROS_INFO("O robo chegou no GOAL!");
            return 0;
        }

        ROS_INFO("ESTADO = %d", estado);
        ROS_INFO("v1=%lg v2=%lg", v1, v2);
        ROS_INFO("beta=%lg alfa=%lg erro_ang=%lg erro_lin=%lg", beta, alfa, erro_ang, erro_lin);

        // CONTINUAR LOGICA
        // criar funcao que recebe goal e retorna velocidades?
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