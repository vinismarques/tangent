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

//Variaveis Globais Para Leitura de Dados
nav_msgs::Odometry current_pose;
sensor_msgs::LaserScan current_laser;
double v1 = 0.0, v2 = 0.0; // Velocidades
double alfa, beta;
int estado = 0;
bool at_goal = false;
bool seguir_esquerda = false, seguir_direita = false, rotate2follow = true, create_rotation_target = true;
double prec_ang = 0.1, prec_lin = 0.1;
double k1 = 0.4, k2 = 0.06;
double orientacao; // Orientacao do robo
double erro_lin, erro_ang;
double difx, dify;
double localx, localy; // Alvo local
double robox, roboy;
double angulo = -45;

//Funcao Callback do Laser
void lasercallback(const sensor_msgs::LaserScan::ConstPtr &laser)
{
    current_laser = *laser;

    return;
}

//Funcao Callback da Pose
void posecallback(const nav_msgs::Odometry::ConstPtr &pose)
{
    current_pose = *pose;

    return;
}

//Funcao Modulo
double modulo(double valor)
{
    return valor >= 0.0 ? valor : -valor;
}

//Funcao que transforma orientacao de radianos para graus (0 a 360)
double graus(double orientacao)
{
    orientacao = orientacao * 180 / M_PI; // Converte de radianos para graus

    if (orientacao <= -180 || orientacao > 0)
    {

        return modulo(orientacao);
    }
    if (orientacao < 0)
    {

        return orientacao + 360;
    }
}

// Funcao que rotaciona de acordo com diferenca de angulos (goal-robo)
double rotaciona(double erro_ang, double prec_ang, double k2)
{
    if (modulo(erro_ang) > prec_ang)
    {
        // Define sentido de rotacao
        // Escolhe o com menor diferenca
        if (erro_ang > 0 && erro_ang <= 180) // Rotaciona no sentido anti-horario
        {

            return modulo(erro_ang) * k2;
        }
        else // Rotaciona no sentido horario
        {

            return -modulo(erro_ang) * k2;
        }
    }
    else
    { // Precisao angular atingida, para de rotacionar

        return 0.0;
    }
}

// Funcao que trata o movimento linear
double anda(double erro_lin, double prec_lin, double k1)
{
    if (erro_lin >= prec_lin)
    { // Realimenta o erro multiplicado por um ganho

        return modulo(erro_lin) * k1;
    }
    else
    { // Precisao linear atingida, para de andar

        return 0.0;
    }
}

// Funcao movetogoal
void movetogoal(double localx, double localy, double robox, double roboy)
{
    if (modulo(erro_ang) >= prec_ang)
    { // Continua movimento angular
        v1 = 0;
        v2 = rotaciona(erro_ang, prec_ang, k2);
    }
    else
    {
        v2 = 0.0;
        if (modulo(erro_lin) < prec_lin)
        { // Chegou no goal, vai para o proximo estado
            v1 = 0.0;
            at_goal = true;
            estado = 1;
            // ROS_INFO("ERRO LIN = %lg", modulo(erro_lin)); // debugging
        }
        else
        { // Continua movimento linear
            v1 = anda(erro_lin, prec_lin, k1);
        }
    }

    /*
    v2 = rotaciona(erro_ang, prec_ang, k2);
    v1 = anda(erro_lin, prec_lin, k1);

    if (modulo(erro_lin) < prec_lin && modulo(erro_ang) < prec_ang) // Chegou no goal, vai para o proximo estado
    {
        //     v1 = 0.0;
        at_goal = true;
        //     // ROS_INFO("ERRO LIN = %lg", modulo(erro_lin)); // debugging
    }*/

    // ROS_INFO("beta=%lg alfa=%lg erro_ang=%lg erro_lin=%lg", beta, alfa, erro_ang, erro_lin); // debugging
}

// Define um target qualquer para rotacionar o robo
void define_rotation_target(double x)
{
    localx = robox + 0.1 * cos((beta + x) * M_PI / 180);
    localy = roboy + 0.1 * sin((beta + x) * M_PI / 180);
}

double segue_parede_esquerda(double esq90, double esq45, double frente, double dir45, double dir90, double tol)
{

    if (rotate2follow == true)
    {
        v1 = 0;
        if (create_rotation_target == true)
        {
            define_rotation_target(angulo); // Rotaciona 90 graus para a direita
            create_rotation_target = false;
            ROS_INFO("CREATING ROTATION TARGET");
        }
        else
        {

            v2 = rotaciona(erro_ang, prec_ang, k2);
            if (modulo(erro_ang) < prec_ang) // Ja girou 90 graus para a direita
            {
                rotate2follow = false;
            }
        }
    }
    else
    {
        ROS_INFO("esq90=%lg esq45=%lg frente=%lg", esq90, esq45, frente);
        if (esq90 < tol + prec_ang)
        {
            v1 = 0.3;
            v2 = -modulo(esq90 - tol) * 3;
        }
        else if (esq90 > tol + prec_ang)
        {
            v1 = 0.3;
            v2 = modulo(esq90 - tol) * 3;
        }
        else
        {
            v1 = 0.3;
            v2 = 0.0;
        }
        if (frente <= tol)
        {
            ROS_WARN("FRENTE!!! %lg", frente);
            rotate2follow = true;
            create_rotation_target = true;
            angulo = -45;
        }
        else if (esq45 <= tol)
        {
            rotate2follow = true;
            create_rotation_target = true;
            angulo = -20;
            ROS_WARN("ESQ 45!!! %lg", frente);
        }
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Path"); // Inicializacao do Nodo

    ros::NodeHandle n;

    // Configuracao do topico a ser publicado
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Configuracao dos topicos a serem lidos
    ros::Subscriber sub = n.subscribe("/base_scan", 10, lasercallback);
    ros::Subscriber sub1 = n.subscribe("/base_pose_ground_truth", 10, posecallback);

    // Define a frequencia do no
    ros::Rate loop_rate(20);

    // Declaracoes
    geometry_msgs::Twist speed_create; // Comando de Velocidade
    double goalx, goaly;               // Alvo global
    double tol = 0.4;

    // Faz Leitura dos Parametros para o GOAL
    if (argc == 1)
    {
        printf("Definido Padrao Aleatorio\n");
        srand(time(NULL));
        goalx = double(rand() % 200) / 10.0;
        srand(time(NULL));
        goaly = double(rand() % 150) / 10.0;

        // Printa o GOAL no terminal
        ROS_INFO("GOAL aleatorio: x=%lf, y=%lf", goalx, goaly);
    }
    else
    {
        if (argc == 3)
        {
            goalx = atof(argv[1]);
            goaly = atof(argv[2]);
            printf("Definido GOAL %lf %lf\n", goalx, goaly);
        }
        else
        {
            printf("Error in Goal\n");
            return -1;
        }
    }

    // Inicializa goal local com valores do goal global
    localx = goalx;
    localy = goaly;

    sleep(1);        // Aguarda 1 seg antes de iniciar para evitar warnings de "Quaternion Not Properly Normalized"
    ros::spinOnce(); // Evita ler sensor antes de o ter inicializado

    //Loop Principal
    while (ros::ok())
    {
        // As posicoes observadas no laser sao invertidas, entao: -90=[900] -45=[720] 0=[540] +45=[360] +90=[180]
        float esq90 = current_laser.ranges[900];
        float esq45 = current_laser.ranges[720];
        float frente = current_laser.ranges[540];
        float dir45 = current_laser.ranges[360];
        float dir90 = current_laser.ranges[180];

        orientacao = tf::getYaw(current_pose.pose.pose.orientation); // getYaw recebe quaternion e converte para radianos
        beta = graus(orientacao);                                    // Converte a orientacao do robo para graus

        robox = current_pose.pose.pose.position.x; // Posicao atual do robo em x
        roboy = current_pose.pose.pose.position.y; // Posicao atual do robo em y

        difx = localx - robox; // Diferenca entre posicao x do goal e do robo
        dify = localy - roboy; // Diferenca entre posicao y do goal e do robo

        alfa = atan2(dify, difx); // Angulo alfa pode ser calculado com a tangente
        alfa = graus(alfa);       // Converte de radianos para graus

        erro_lin = sqrt(difx * difx + dify * dify); // Define o erro linear
        erro_ang = alfa - beta;                     // Define o erro angular

        if (estado == 0)
        {
            movetogoal(localx, localy, robox, roboy);

            // Muda para rotina de contorno de obstaculo
            if (frente < tol + 0.1 || esq45 < tol || dir45 < tol)
            {
                estado = 1;
            }
            ROS_INFO("FRENTE = %f", frente);
        }

        if (at_goal == true)
        {
        }

        // Estado que faz o contorno de obstaculos
        if (estado == 1)
        {
            segue_parede_esquerda(esq90, esq45, frente, dir45, dir90, tol);
        }

        // ROS_INFO("ESTADO = %d", estado);
        ROS_INFO("v1=%lg v2=%lg", v1, v2);

        // Estado final, encerra o nodo
        if (estado == 4)
        {
            ROS_INFO("O robo chegou no GOAL!");
            return 0;
        }

        // CONTINUAR LOGICA
        // criar funcao que recebe goal e retorna velocidades?
        // usar essa funcao para passar goals intermediarios
        // usar RRT (usar goal como aleatorio)?

        // Envia Sinal de Velocidade
        speed_create.linear.x = v1;
        speed_create.angular.z = v2;
        vel_pub.publish(speed_create);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}