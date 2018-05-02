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
double tol = 0.4;          // Distancia de tolerancia do robo ate a parede
double alfa, beta;         // alfa = angulo do goal local em relacao ao robo. beta = angulo do robo em relacao ao frame
int estado = 0;            // Variavel utilizada na maquina de estados
bool seguir_esquerda = false, seguir_direita = false, rotate2follow = true, create_rotation_target = true;
double prec_ang = 0.1, prec_lin = 0.1;    // Precisoes exigidas que os controladores respeitem
double k1 = 0.4, k2 = 0.06;               // k1: ganho do controle linear; k2: ganho do controle angular
double orientacao;                        // Orientacao do robo em radianos
double erro_lin, erro_ang;                // Erro linear e erro angular
double difx, dify;                        // Diferenca da posicao do robo e do goal local
double localx, localy;                    // Goal local
double robox, roboy;                      // Valores em x e y da posicao do robo
double dist;                              // Distancia entre o robo e goal global
double angulo = -45;                      // Valor inicial da variavel angulo. Utilizada para criar targets de rotacao
float esq90, esq45, frente, dir45, dir90; // Variaveis de leitura do sensor laser

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
    if (modulo(erro_ang) > prec_ang) // Define sentido de rotacao. Escolhe o com menor diferenca
    {
        if (erro_ang > 0 && erro_ang <= 180) // Rotaciona no sentido anti-horario
        {

            return modulo(erro_ang) * k2;
        }
        else // Rotaciona no sentido horario
        {

            return -modulo(erro_ang) * k2;
        }
    }
    else // Precisao angular atingida, para de rotacionar
    {
        return 0.0;
    }
}

// Funcao que trata o movimento linear
double anda(double erro_lin, double prec_lin, double k1)
{
    if (erro_lin >= prec_lin) // Realimenta o erro multiplicado por um ganho
    {

        return modulo(erro_lin) * k1;
    }
    else // Precisao linear atingida, para de andar
    {

        return 0.0;
    }
}

// Funcao movetogoal
void movetogoal(double localx, double localy, double robox, double roboy)
{
    if (modulo(erro_ang) >= prec_ang) // Continua movimento angular
    {
        v1 = 0;
        v2 = rotaciona(erro_ang, prec_ang, k2);
    }
    else
    {
        v2 = 0.0;

        if (modulo(erro_lin) < prec_lin) // Chegou no goal, vai para o estado final
        {
            v1 = 0.0;
            estado = 2;
        }
        else // Continua movimento linear
        {
            v1 = anda(erro_lin, prec_lin, k1);
        }
    }

    // Checa se o goal e alcancavel, se nao, retorna que chegou o mais proximo possivel
    if (frente < tol && dist < 1.5)
    {
        v1 = 0;
        v2 = 0;
        estado = 2;
        ROS_WARN("O robo chegou o mais proximo possivel do goal.");
    }

    // ROS_INFO("beta=%lg alfa=%lg erro_ang=%lg erro_lin=%lg", beta, alfa, erro_ang, erro_lin); // Debugging
}

// Define um target somente para rotacionar o robo baseado no argumento de angulo recebido
void define_rotation_target(double x)
{
    localx = robox + 0.1 * cos((beta + x) * M_PI / 180);
    localy = roboy + 0.1 * sin((beta + x) * M_PI / 180);
}

// Rotina que faz o robo seguir a parede que esta a sua esquerda
double segue_parede_esquerda(double tol)
{
    // Rotina de rotacao forcada. Rotaciona antes de seguir. Possibilita mudancas bruscas de sentido
    if (rotate2follow == true)
    {
        v1 = 0; // Somente rotaciona, sem movimento linear

        if (create_rotation_target == true) // Se o target nao estiver criado, criar um novo
        {
            define_rotation_target(angulo); // Gera target x e y de acordo com o angulo desejado
            create_rotation_target = false; // Garante que não vai criar um novo target sem necessidade
            // ROS_INFO("CREATING ROTATION TARGET"); // Debugging
        }
        else // Se o target já existe, rotacionar ate alcanca-lo
        {

            v2 = rotaciona(erro_ang, prec_ang, k2); // Usa os pontos criados por define_rotation_target()
            if (modulo(erro_ang) < prec_ang)        // Se satisfazer, significa que a precisao ja foi alcancada
            {
                rotate2follow = false; // Vai pra rotina automatica
            }
        }
    }
    // Rotina de rotacao automatica. Usada para fazer o robo seguir a parede mantendo uma distancia fixa
    else
    {
        v1 = 0.3;
        // ROS_INFO("esq90=%lg esq45=%lg frente=%lg", esq90, esq45, frente); // Debugging

        if (esq90 < tol) // Corrige para a direita
        {
            v2 = -modulo(esq90 - tol) * 3;
        }
        else if (esq90 > tol) // Corrige para a esquerda
        {
            v2 = modulo(esq90 - tol) * 3;
        }
        else // Somente movimento linear
        {
            v2 = 0.0;
        }
        if (frente <= tol) // Obstaculo a frente. Rotaciona 45 graus para a direita
        {
            rotate2follow = true;          // Habilita rotina de rotacao forcada
            create_rotation_target = true; // Habilita criar um novo target
            angulo = -45;                  // Valor de rotacao usado para criar novo target
            // ROS_WARN("Obstaculo a frente. Distancia: %lg", frente); // Debugging
        }
        else if (esq45 <= tol) // Osbtaculo imediatamente a esquerda. Rotaciona 20 graus para a direita
        {
            rotate2follow = true;          // Habilita rotina de rotacao forcada
            create_rotation_target = true; // Habilita criar um novo target
            angulo = -20;                  // Valor de rotacao usado para criar novo target
            // ROS_WARN("Obstaculo 45 graus na esquerda. Distancia: %lg", esq45); // Debugging
        }
    }

    // Checa se o goal e alcancavel, se nao, retorna que chegou o mais proximo possivel
    if (dist < 1.5 && esq90 < 2 * tol)
    {
        v1 = 0;
        v2 = 0;
        estado = 2;
        ROS_INFO("O robo chegou o mais proximo possivel do goal."); // Debugging
    }
}

// Rotina que faz o robo seguir a parede que esta a sua direita
double segue_parede_direita(double tol)
{
    // Rotina de rotacao forcada. Rotaciona antes de seguir. Possibilita mudancas bruscas de sentido
    if (rotate2follow == true)
    {
        v1 = 0; // Somente rotaciona, sem movimento linear

        if (create_rotation_target == true) // Se o target nao estiver criado, criar um novo
        {
            define_rotation_target(angulo); // Gera target x e y de acordo com o angulo desejado
            create_rotation_target = false; // Garante que não vai criar um novo target sem necessidade
            // ROS_INFO("CREATING ROTATION TARGET"); // Debugging
        }
        else // Se o target já existe, rotacionar ate alcanca-lo
        {

            v2 = rotaciona(erro_ang, prec_ang, k2); // Usa os pontos criados por define_rotation_target()
            if (modulo(erro_ang) < prec_ang)        // Se satisfazer, significa que a precisao ja foi alcancada
            {
                rotate2follow = false; // Vai pra rotina automatica
            }
        }
    }
    // Rotina de rotacao automatica. Usada para fazer o robo seguir a parede mantendo uma distancia fixa
    else
    {
        v1 = 0.3;
        // ROS_INFO("dir90=%lg dir45=%lg frente=%lg", dir90, dir45, frente); // Debugging

        if (dir90 < tol) // Corrige para a esquerda
        {
            v2 = modulo(dir90 - tol) * 3;
        }
        else if (dir90 > tol) // Corrige para a direita
        {
            v2 = -modulo(dir90 - tol) * 3;
        }
        else // Somente movimento linear
        {
            v2 = 0.0;
        }
        if (frente <= tol) // Obstaculo a frente. Rotaciona 45 graus para a esquerda
        {
            rotate2follow = true;                                   // Habilita rotina de rotacao forcada
            create_rotation_target = true;                          // Habilita criar um novo target
            angulo = 45;                                            // Valor de rotacao usado para criar novo target
            ROS_WARN("Obstaculo a FRENTE. Distancia: %lg", frente); // Debugging
        }
        else if (dir45 <= tol) // Osbtaculo imediatamente a direita. Rotaciona 20 graus para a esquerda
        {
            rotate2follow = true;                                             // Habilita rotina de rotacao forcada
            create_rotation_target = true;                                    // Habilita criar um novo target
            angulo = 20;                                                      // Valor de rotacao usado para criar novo target
            ROS_WARN("Obstaculo 45 graus na direita. Distancia: %lg", dir45); // Debugging
        }
        ROS_INFO("dir90=%lg dir45=%lg frent=%lg", dir90, dir45, frente);
        ROS_INFO("v1=%lg v2=%lg", v1, v2);
    }

    // Checa se o goal e alcancavel, se nao, retorna que chegou o mais proximo possivel
    if (dist < 1.5 && dir90 < 2 * tol)
    {
        v1 = 0;
        v2 = 0;
        estado = 2;
        ROS_INFO("O robo chegou o mais proximo possivel do goal."); // Debugging
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
    double alfa_laser;                 // Relacao de angulo do feixe de laser com o goal global
    double erro_ang_global;            // Erro angular entre o feixe de laser e o goal global

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
        // Necessário atualizar a cada iteracao, por isso esta dentro do while
        esq90 = current_laser.ranges[900];
        esq45 = current_laser.ranges[720];
        frente = current_laser.ranges[540];
        dir45 = current_laser.ranges[360];
        dir90 = current_laser.ranges[180];

        // Informacoes do robo
        orientacao = tf::getYaw(current_pose.pose.pose.orientation); // getYaw recebe quaternion e converte para radianos
        beta = graus(orientacao);                                    // Converte a orientacao do robo para graus
        robox = current_pose.pose.pose.position.x;                   // Posicao atual do robo em x
        roboy = current_pose.pose.pose.position.y;                   // Posicao atual do robo em y

        // Distancia do robo ate o goal global
        dist = sqrt((goalx - robox) * (goalx - robox) + (goaly - roboy) * (goaly - roboy));

        // Relacoes entre robo e goal local
        difx = localx - robox;                      // Diferenca entre posicao x do goal local e do robo
        dify = localy - roboy;                      // Diferenca entre posicao y do goal local e do robo
        alfa = atan2(dify, difx);                   // Angulo alfa pode ser calculado com a tangente
        alfa = graus(alfa);                         // Converte de radianos para graus
        erro_lin = sqrt(difx * difx + dify * dify); // Define o erro linear
        erro_ang = alfa - beta;                     // Define o erro angular

        // Estado que cuida do movimento ate o goal
        if (estado == 0)
        {
            movetogoal(localx, localy, robox, roboy);

            // Se detecta obstaculo, muda para rotina de contorno de obstaculo
            if (frente < tol + 0.1 || esq45 < tol || dir45 < tol)
            {
                ROS_INFO("Obstaculo detectado. Iniciando rotina de contorno.");
                estado = 1;
            }

            // ROS_INFO("Frente=%lg Dist=%lg", frente, dist); // Debugging
        }

        // Estado que faz o contorno de obstaculos
        if (estado == 1)
        {
            segue_parede_esquerda(tol);
            // segue_parede_direita(tol);

            // Para laser frontal. Para outros seria necessario multiplicar por sin e cos do angulo relativo
            alfa_laser = graus(atan2(goaly - roboy, goalx - robox));
            erro_ang_global = alfa_laser - beta; // Somar/subtrair beta para fazer o mesmo com outros angulos

            // Se detecta que o caminho ate o goal esta limpo, muda para rotina movetogoal
            if (modulo(erro_ang_global) <= 4) // Threshold do erro do angulo
            {
                // ROS_INFO("frente=%lg dist=%lg", alfa_laser, erro_ang_global);
                if (frente >= dist) // Vai para a rotina movetogoal
                {
                    ROS_INFO("Caminho ate o goal livre. Iniciando rotina movetogoal.");
                    localx = goalx;
                    localy = goaly;
                    estado = 0;
                }
                // ROS_INFO("Validou entrada no threshold do erro do angulo.");
            }
        }

        // ROS_INFO("ESTADO = %d", estado); // Debugging
        // ROS_INFO("v1=%lg v2=%lg", v1, v2); // Debugging

        // Estado final, encerra o nodo
        if (estado == 2)
        {
            ROS_INFO("O robo chegou ao GOAL!");
            return 0;
        }

        // Envia Sinal de Velocidade
        speed_create.linear.x = v1;
        speed_create.angular.z = v2;
        vel_pub.publish(speed_create);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}