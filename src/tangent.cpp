//
// TO-DO:   corrigir sentido de rotacao (escolher menor lado)
//          rotacao indo pro lado oposto causando overshoot!!!
//
//
//          Revisar codigo passo a passo
//

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
#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <iomanip>
#include <map>
#include <set>
#include <array>
#include <utility>
#include <queue>
#include <tuple>
#include <algorithm>

using namespace std;

#define GRID_RES_X 2 // cells per meters (cell density)
#define GRID_RES_Y 2

const int GRID_WIDTH = 43,
          GRID_HEIGHT = 33;

// char gMap[GRID_HEIGHT][GRID_WIDTH];

//Variaveis Globais Para Leitura de Dados
nav_msgs::Odometry current_pose;
sensor_msgs::LaserScan current_laser;
double v1 = 0.0, v2 = 0.0; // Velocidades
double tol = 0.05;         // Distancia de tolerancia do robo ate a parede
double alfa, beta;         // alfa = angulo do goal local em relacao ao robo. beta = angulo do robo em relacao ao frame
int estado = 0;            // Variavel utilizada na maquina de estados
int mapRobox, mapRoboy,
    mapGoalx, mapGoaly; // Variavel da posicao do mapa
int step = 0;           // Passo atual da trajetoria encontrada
bool seguir_esquerda = false, seguir_direita = false, rotate2follow = true, create_rotation_target = true;
double prec_ang = 0.5, prec_lin = 0.03;   // Precisoes exigidas que os controladores respeitem
double kp1 = 5.0,                         // kp1: ganho proporcional do controle linear
    kp2 = 0.06;                           // kp2: ganho proporcional do controle angular
double ki1 = 0.0,                         // ki1: ganho integal do controle linear
    ki2 = 0.0,                            // ki2: ganho integral do controle angular
    integral1 = 0,                        // integral do erro linear
    integral2 = 0,                        // integral do erro angular
    dt = 0.1;                             // constante de tempo
double orientacao;                        // Orientacao do robo em radianos
double erro_lin, erro_ang;                // Erro linear e erro angular
double difx, dify;                        // Diferenca da posicao do robo e do goal local
double localx, localy;                    // Goal local
double robox, roboy;                      // Valores em x e y da posicao do robo
double dist;                              // Distancia entre o robo e goal global
double angulo = -45;                      // Valor inicial da variavel angulo. Utilizada para criar targets de rotacao
float esq90, esq45, frente, dir45, dir90; // Variaveis de leitura do sensor laser
bool laserReady = false, odomReady = false;

//Funcao Callback do Laser
void lasercallback(const sensor_msgs::LaserScan::ConstPtr &laser)
{
    current_laser = *laser;
    laserReady = true;

    return;
}

//Funcao Callback da Pose
void posecallback(const nav_msgs::Odometry::ConstPtr &pose)
{
    current_pose = *pose;
    odomReady = true;

    return;
}

/*
void loadMap(string fileName)
{
    FILE *f = fopen(fileName.c_str(), "r");
    int v;
    for (unsigned i = 0; i < GRID_HEIGHT; i++)
    {
        for (unsigned j = 0; j < GRID_WIDTH; j++)
        {
            fscanf(f, "%d,", &v);
            gMap[i][j] = v;
            // printf("%d ", gMap[i][j]);
        }
        // printf("\n");
    }
    fclose(f);
    // printf("\n\n");
}
*/

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
double rotaciona(double erro_ang, double prec_ang, double kp2)
{
    if (modulo(erro_ang) > prec_ang) // Define sentido de rotacao. Escolhe o com menor diferenca
    {
        if (erro_ang > 0 && erro_ang <= 180) // Rotaciona no sentido anti-horario
        {
            integral2 += modulo(erro_ang * dt);
            return modulo(erro_ang) * (kp2 + ki2 * integral2);
        }
        else // Rotaciona no sentido horario
        {
            // cout << "\nENTROU! Rotacionando anti-horario\n";
            integral2 += modulo(erro_ang * dt);
            return -modulo(erro_ang) * (kp2 + ki2 * integral2);
        }
    }
    else // Precisao angular atingida, para de rotacionar
    {
        integral2 = 0;
        return 0.0;
    }
}

// Funcao que trata o movimento linear
double anda(double erro_lin, double prec_lin, double kp1)
{
    if (erro_lin >= prec_lin) // Realimenta o erro multiplicado por um ganho
    {
        integral1 += modulo(erro_lin * dt);
        return modulo(erro_lin) * (kp1 + ki1 * integral1);
    }
    else // Precisao linear atingida, para de andar
    {
        integral1 = 0;
        return 0.0;
    }
}

// Funcao movetogoal
void movetogoal(double localx, double localy, double robox, double roboy)
{
    if (modulo(erro_ang) >= prec_ang) // Continua movimento angular
    {
        v1 = 0;
        v2 = rotaciona(erro_ang, prec_ang, kp2);
    }
    else
    {
        v2 = 0.0;

        if (modulo(erro_lin) < prec_lin) // Chegou no goal, vai para o estado final
        {
            v1 = 0.0;
            v2 = 0;
            estado = 2;
        }
        else // Continua movimento linear
        {
            v1 = anda(erro_lin, prec_lin, kp1);
        }
    }

    // Checa se o goal e alcancavel, se nao, retorna que chegou o mais proximo possivel
    if ((frente || esq45 || dir45) < tol && dist < 1.5)
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
/*
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

            v2 = rotaciona(erro_ang, prec_ang, kp2); // Usa os pontos criados por define_rotation_target()
            if (modulo(erro_ang) < prec_ang)         // Se satisfazer, significa que a precisao ja foi alcancada
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
        else if (esq45 <= tol || dir45 <= tol / 2) // Osbtaculo imediatamente a esquerda. Rotaciona 20 graus para a direita
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

            v2 = rotaciona(erro_ang, prec_ang, kp2); // Usa os pontos criados por define_rotation_target()
            if (modulo(erro_ang) < prec_ang)         // Se satisfazer, significa que a precisao ja foi alcancada
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
*/
void calcula_erro_ang()
{
    // Descomentar este trecho se rotacao estiver funcionando mal
    // if (alfa < 0 && orientacao < 0)
    // {
    //     alfa = (2 * M_PI + alfa);
    //     orientacao = (2 * M_PI + orientacao);
    // }
    // else
    // {
    //     if (alfa < 0)
    //     {
    //         alfa = (2 * M_PI + alfa);
    //     }
    //     else if (orientacao < 0)
    //     {
    //         orientacao = (2 * M_PI + orientacao);
    //     }
    // }

    erro_ang = (alfa - orientacao) * 180 / M_PI;

    // Corrige de acordo com circulo trigonometrico
    if (erro_ang > 180)
    {
        erro_ang = erro_ang - 360;
    }
    if (erro_ang < -180)
    {
        erro_ang = erro_ang + 360;
    }
}

struct GridLocation
{
    int x, y;
};

struct SquareGrid
{
    static array<GridLocation, 4> DIRS;

    int width, height;
    set<GridLocation> walls;
    set<GridLocation> notvisited;

    SquareGrid(int width_, int height_)
        : width(width_), height(height_) {}

    bool in_bounds(GridLocation id) const
    {
        return 0 <= id.x && id.x < width && 0 <= id.y && id.y < height;
    }

    bool passable(GridLocation id) const
    {
        return walls.find(id) == walls.end();
    }

    vector<GridLocation> neighbors(GridLocation id) const
    {
        vector<GridLocation> results;

        for (GridLocation dir : DIRS)
        {
            GridLocation next{id.x + dir.x, id.y + dir.y};
            if (in_bounds(next) && passable(next))
            {
                results.push_back(next);
            }
        }

        // if ((id.x + id.y) % 2 == 0) {
        //   // aesthetic improvement on square grids
        //   reverse(results.begin(), results.end());
        // }

        return results;
    }
};

array<GridLocation, 4> SquareGrid::DIRS =
    {GridLocation{1, 0}, GridLocation{0, -1}, GridLocation{-1, 0}, GridLocation{0, 1}};

bool operator==(GridLocation a, GridLocation b)
{
    return a.x == b.x && a.y == b.y;
}

bool operator!=(GridLocation a, GridLocation b)
{
    return !(a == b);
}

bool operator<(GridLocation a, GridLocation b)
{
    return tie(a.x, a.y) < tie(b.x, b.y);
}

basic_iostream<char>::basic_ostream &operator<<(basic_iostream<char>::basic_ostream &out, const GridLocation &loc)
{
    out << '(' << loc.x << ',' << loc.y << ')';
    return out;
}

// This outputs a grid. Pass in a distances map if you want to print
// the distances, or pass in a point_to map if you want to print
// arrows that point to the parent location, or pass in a path vector
// if you want to draw the path.
template <class Graph>
void draw_grid(const Graph &graph, int field_width,
               map<GridLocation, double> *distances = nullptr,
               map<GridLocation, GridLocation> *point_to = nullptr,
               vector<GridLocation> *path = nullptr)
{
    for (int y = 0; y != graph.height; ++y)
    {
        for (int x = 0; x != graph.width; ++x)
        {
            GridLocation id{x, y};
            cout << left << setw(field_width);
            if (graph.walls.find(id) != graph.walls.end())
            {
                cout << string(field_width, '#');
            }
            else if (point_to != nullptr && point_to->count(id))
            {
                GridLocation next = (*point_to)[id];
                if (next.x == x + 1)
                {
                    cout << "> ";
                }
                else if (next.x == x - 1)
                {
                    cout << "< ";
                }
                else if (next.y == y + 1)
                {
                    cout << "v ";
                }
                else if (next.y == y - 1)
                {
                    cout << "^ ";
                }
                else
                {
                    cout << "* ";
                }
            }
            else if (distances != nullptr && distances->count(id))
            {
                cout << (*distances)[id];
            }
            else if (path != nullptr && find(path->begin(), path->end(), id) != path->end())
            {
                cout << '@';
            }
            else
            {
                cout << '.';
            }
        }
        cout << '\n';
    }
}

// void add_rect(SquareGrid &grid, int x1, int y1, int x2, int y2)
// {
//     for (int x = x1; x < x2; ++x)
//     {
//         for (int y = y1; y < y2; ++y)
//         {
//             grid.walls.insert(GridLocation{x, y});
//         }
//     }
// }

///////////////////////////////////////

void loadMap(SquareGrid &grid)
{
    string fileName = "/home/vinicius/at_home_ws/src/tangent/world/map";
    FILE *f = fopen(fileName.c_str(), "r");
    int v;
    for (int j = 0; j < GRID_HEIGHT; j++)
    {
        for (int i = 0; i < GRID_WIDTH; i++)
        {
            fscanf(f, "%d,", &v);
            if (v == 2)
            {
                grid.walls.insert(GridLocation{i, j});
            }
            if (v == 0)
            {
                grid.notvisited.insert(GridLocation{i, j});
            }
        }
        // printf("\n");
    }
    fclose(f);
    // printf("\n\n");
}

SquareGrid make_diagram()
{
    SquareGrid grid(43, 33);
    loadMap(grid);

    // cout << grid.walls.size() << '\n';
    // grid.walls.erase({0, 0});
    // cout << grid.walls.size() << '\n';
    cout << grid.notvisited.size() << '\n';

    return grid;
}

template <typename Location>
vector<Location> reconstruct_path(
    Location start, Location goal,
    map<Location, Location> came_from)
{
    vector<Location> path;
    Location current = goal;
    while (current != start)
    {
        path.push_back(current);
        current = came_from[current];
    }
    path.push_back(start); // optional
    reverse(path.begin(), path.end());
    return path;
}

template <typename Location, typename Graph>
map<Location, Location>
breadth_first_search(Graph graph, Location start, Location goal)
{
    queue<Location> frontier;
    frontier.push(start);

    map<Location, Location> came_from;
    came_from[start] = start;

    while (!frontier.empty())
    {
        Location current = frontier.front();
        frontier.pop();

        if (current == goal)
        {
            break;
        }

        for (Location next : graph.neighbors(current))
        {
            if (came_from.find(next) == came_from.end())
            {
                frontier.push(next);
                came_from[next] = current;
            }
        }
    }
    return came_from;
}

double map2pos(double i)
{
    return i / 2 + 0.25;
}

//////////////////////////////// globais custom ////////////////////////////////
SquareGrid grid = make_diagram(); // Declara o mapa (grid) como variavel global
vector<GridLocation> path;        // Declara o caminho como variavel global

void nextstep()
{
    ROS_WARN("step:%d path.x:%d path.y:%d", step, path[step].x, path[step].y);
    localx = map2pos(path[step].x);
    localy = map2pos(path[step].y);
    ROS_INFO("localx:%lg localy:%lg", localx, localy);
    step++;
    return;
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
    if (argc == 2)
    {
        printf("Iniciando limpeza do chao\n");
        // loadMap(argv[1]);

        // for (signed i = (GRID_HEIGHT - 1); i > -1; i--)
        // {
        //     for (unsigned j = 0; j < GRID_WIDTH; j++)
        //     {
        //         // printf("%d ", gMap[i][j]);
        //     }
        //     // printf("\n");
        // }

        //////////////////////////////////////////////////////////////
        GridLocation start{16, 16};
        GridLocation goal{17, 30};
        // SquareGrid grid = make_diagram();
        // printf("%d\n", gMap[12][7]); // remover depois de testes
        auto came_from = breadth_first_search(grid, start, goal);
        draw_grid(grid, 2, nullptr, &came_from);

        cout << '\n';
        path = reconstruct_path(start, goal, came_from);
        draw_grid(grid, 2, nullptr, nullptr, &path);
        // return 0;
        //////////////////////////////////////////////////////////////

        srand(time(NULL));
        // goalx = double(rand() % 150) / 10.0;
        // goaly = double(rand() % 150) / 10.0;

        // Printa o GOAL no terminal
        // ROS_INFO("GOAL aleatorio: x=%lf, y=%lf", goalx, goaly);

        // cout << grid.notvisited.size() << '\n';
    }
    else
    {
        if (argc == 4)
        {
            goalx = atof(argv[2]);
            goaly = atof(argv[3]);
            printf("Definido GOAL %lf %lf\n", goalx, goaly);
        }
        else
        {
            printf("Error in Goal\n");
            return -1;
        }
    }

    // Inicializa goal local com valores do goal global
    // localx = goalx;
    // localy = goaly;
    goalx = map2pos(path[step].x);
    goaly = map2pos(path[step].y);
    // localx = map2pos(path[step].x);
    // localy = map2pos(path[step].y);
    // ROS_INFO("%d", step);
    // step++;
    // ROS_INFO("%d", step);
    nextstep();
    // ROS_INFO("%lg %lg", localx, localy);

    // Wait sensors be ready
    while (!laserReady || !odomReady)
    {
        ros::spinOnce();
        loop_rate.sleep();
        if (!ros::ok())
            break;
    }

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

        GridLocation grid_current{int(robox * 2), int(roboy * 2)}; // Posicao atual na grid
        GridLocation grid_goal{int(goalx * 2), int(goaly * 2)};    // Goal global na grid
        GridLocation grid_local{int(localx * 2), int(localx * 2)}; // Goal local na grid

        // cout << grid_current.x << "\n";

        // Distancia do robo ate o goal global
        dist = sqrt((goalx - robox) * (goalx - robox) + (goaly - roboy) * (goaly - roboy));

        // Relacoes entre robo e goal local
        difx = localx - robox;    // Diferenca entre posicao x do goal local e do robo
        dify = localy - roboy;    // Diferenca entre posicao y do goal local e do robo
        alfa = atan2(dify, difx); // Angulo alfa pode ser calculado com a tangente
        // alfa = atan2(modulo(dify), modulo(difx));   // Angulo alfa pode ser calculado com a tangente
        ROS_INFO("ANTES: orientacao=%lg alfa=%lg alfa-ori=%lg", orientacao, alfa, alfa - orientacao);

        // Define o erro angular
        // if (alfa < 0 && orientacao > 0) {
        //     alfa = (2 * M_PI + alfa);
        //     // cout << 2 * M_PI + alfa << '\n';
        // }

        // if (alfa < 0 && orientacao < 0)
        // {
        //     alfa = (2 * M_PI + alfa);
        //     orientacao = (2 * M_PI + orientacao);
        // }
        // else
        // {
        //     if (alfa < 0)
        //     {
        //         alfa = (2 * M_PI + alfa);
        //     }
        //     else if (orientacao < 0)
        //     {
        //         orientacao = (2 * M_PI + orientacao);
        //     }
        // }
        // erro_ang = (alfa - orientacao) * 180 / M_PI;
        // if (erro_ang > 180){
        //     erro_ang = erro_ang - 360;
        // }
        // if (erro_ang < -180)
        // {
        //     erro_ang = erro_ang + 360;
        // }
        calcula_erro_ang();

        ROS_INFO("DEPOIS: orientacao=%lg alfa=%lg alfa-ori=%lg", orientacao, alfa, alfa - orientacao);

        // ROS_INFO("alfa antes: %lg", alfa);
        alfa = graus(alfa); // Converte de radianos para graus
        // ROS_INFO("alfa em graus: %lg", alfa);
        erro_lin = sqrt(difx * difx + dify * dify); // Define o erro linear
        // erro_ang = alfa - beta;                     // Define o erro angular
        ROS_INFO("erro_ang: %lg", erro_ang);
        ROS_INFO("beta=%lg alfa=%lg erro=%lg", beta, alfa, erro_ang);
        // ROS_INFO("difx=%lg dify=%lg", difx, dify);
        // ROS_INFO("v1=%lg v2=%lg", v1, v2);

        // cout << '\n';

        grid.notvisited.erase(grid_current); // Remove os locais já visitados

        // path.erase();
        // cout << grid.notvisited.size() << "\n";

        // Testing code //
        // grid_local = path[step];
        // ROS_INFO("%lg %lg", robox, roboy);

        // for (size_t i = 0; i < path.size(); i++)
        // {
        //     cout << path[i].x << " " << path[i].y << "\n";
        // }

        if (grid.notvisited.empty())
        {
            // ENCERRAR ROS - VISITOU TODOS OS LOCAIS POSSIVEIS
            return 0;
        }

        // Estado que cuida do movimento ate o goal
        if (estado == 0)
        {
            movetogoal(localx, localy, robox, roboy);

            // Se detecta obstaculo, muda para rotina de contorno de obstaculo
            // if (frente < tol + 0.1 || esq45 < tol || dir45 < tol)
            // {
            //     ROS_INFO("Obstaculo detectado. Iniciando rotina de contorno.");
            //     estado = 1;
            // }

            // ROS_INFO("Frente=%lg Dist=%lg", frente, dist); // Debugging
        }

        // Estado que faz o contorno de obstaculos
        // if (estado == 1)
        // {
        //     segue_parede_esquerda(tol);
        //     // segue_parede_direita(tol);

        //     // Para laser frontal. Para outros seria necessario multiplicar por sin e cos do angulo relativo
        //     alfa_laser = graus(atan2(goaly - roboy, goalx - robox));
        //     erro_ang_global = alfa_laser - beta; // Somar/subtrair beta para fazer o mesmo com outros angulos

        //     // Se detecta que o caminho ate o goal esta limpo, muda para rotina movetogoal
        //     if (modulo(erro_ang_global) <= 4) // Threshold do erro do angulo
        //     {
        //         // ROS_INFO("frente=%lg dist=%lg", alfa_laser, erro_ang_global);
        //         if (frente >= dist && esq45 > tol) // Vai para a rotina movetogoal
        //         {
        //             ROS_INFO("Caminho ate o goal livre. Iniciando rotina movetogoal.");
        //             localx = goalx;
        //             localy = goaly;
        //             estado = 0;
        //         }
        //         // ROS_INFO("Validou entrada no threshold do erro do angulo.");
        //     }
        // }

        // ROS_INFO("ESTADO = %d", estado); // Debugging
        // ROS_INFO("v1=%lg v2=%lg", v1, v2); // Debugging

        // Estado final, encerra o nodo
        if (estado == 2)
        {
            ROS_WARN("O robo chegou ao GOAL!");

            if (step + 1 == path.size())
            {
                return 0;
            }

            // ROS_INFO("Goal antigo: %lg %lg", localx, localy);
            // ROS_INFO("xy: %lg %lg", robox, roboy);

            // goalx = double(rand() % 150) / 10.0;
            // goaly = double(rand() % 150) / 10.0;
            // Printa o GOAL no terminal
            // ROS_INFO("GOAL aleatorio: x=%lf, y=%lf", goalx, goaly);

            nextstep();
            // localx = map2pos(path[step].x);
            // localy = map2pos(path[step].y);
            // step++;
            ROS_INFO("Novo goal local: %lg %lg", localx, localy);

            estado = 0;

            // return 0;
        }

        ROS_INFO("v1=%lg v2=%lg", v1, v2);
        cout << '\n';

        // Envia Sinal de Velocidade
        speed_create.linear.x = v1;
        speed_create.angular.z = v2;
        vel_pub.publish(speed_create);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}