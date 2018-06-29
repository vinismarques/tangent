// Definir rotina de novos targets nao visitados

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
double tol = 0.25;         // Distancia de tolerancia do robo ate a parede
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
int visitado[GRID_WIDTH][GRID_HEIGHT];

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
    // if ((frente) < tol && dist < 1.5)
    // {
    //     v1 = 0;
    //     v2 = 0;
    //     estado = 2;
    //     ROS_WARN("O robo chegou o mais proximo possivel do goal.");
    // }

    // ROS_INFO("beta=%lg alfa=%lg erro_ang=%lg erro_lin=%lg", beta, alfa, erro_ang, erro_lin); // Debugging
}

// Define um target somente para rotacionar o robo baseado no argumento de angulo recebido
void define_rotation_target(double x)
{
    localx = robox + 0.1 * cos((beta + x) * M_PI / 180);
    localy = roboy + 0.1 * sin((beta + x) * M_PI / 180);
}

double calcula_erro_ang()
{
    // Descomentar este trecho se rotacao estiver funcionando mal
    if (alfa < 0 && orientacao < 0)
    {
        alfa = (2 * M_PI + alfa);
        orientacao = (2 * M_PI + orientacao);
    }
    else
    {
        if (alfa < 0)
        {
            alfa = (2 * M_PI + alfa);
        }
        else if (orientacao < 0)
        {
            orientacao = (2 * M_PI + orientacao);
        }
    }

    return (alfa - orientacao) * 180 / M_PI;

    // Corrige de acordo com circulo trigonometrico
    if (erro_ang > 180)
    {
        return erro_ang - 360;
    }
    if (erro_ang < -180)
    {
        return erro_ang + 360;
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

// Carrega mapa e insere paredes no grafo da grid
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
                visitado[i][j] = 1;
            }
            if (v == 0)
            {
                grid.notvisited.insert(GridLocation{i, j});
                visitado[i][j] = 0;
            }
        }
    }
    fclose(f);
}

// Cria grid do mapa
SquareGrid make_diagram()
{
    SquareGrid grid(43, 33);
    loadMap(grid);

    return grid;
}

// Reconstroi caminho encontrado pelo Breadth Search First
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

// Algoritmo de path finding Breadth Search First
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

// Converte localizacao da grid para o real
double map2pos(double i)
{
    return i / 2 + 0.25;
}

// Custom global variables
SquareGrid grid = make_diagram(); // Declara o mapa (grid) como variavel global
vector<GridLocation> path;        // Declara o caminho ate o goal como variavel global
GridLocation grid_goal;
set<GridLocation>::iterator setIt;
GridLocation grid_current;
GridLocation grid_local;

// Busca proximo passo do path planning
void nextstep()
{
    ROS_WARN("step:%d path.x:%d path.y:%d", step, path[step].x, path[step].y);
    localx = map2pos(path[step].x);
    localy = map2pos(path[step].y);
    ROS_INFO("localx:%lg localy:%lg", localx, localy);
    step++;
}

// Busca proxima posicao nao visitada
void nextgoal()
{
    // setar goal como primeira posicao do notvisited e remove-lo depois de visitar
    // cout << "Novo goal: " << *setIt << endl;
    GridLocation goal;
    for (int i = 0; i < GRID_WIDTH; i++)
    {
        for (int j = 0; j < GRID_HEIGHT; j++)
        {
            if (visitado[i][j] == 0)
            {
                goal = {i, j};
                i = GRID_WIDTH;
                j = GRID_HEIGHT;
            }
        }
    }
    cout << "Goal do for: " << goal << endl;

    GridLocation start = grid_current;
    // GridLocation goal{17, 30};
    // GridLocation goal = *setIt;

    auto came_from = breadth_first_search(grid, start, goal);
    draw_grid(grid, 2, nullptr, &came_from);

    cout << '\n';
    path = reconstruct_path(start, goal, came_from);
    draw_grid(grid, 2, nullptr, nullptr, &path);

    setIt++;
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

    // Verifica se e para entrar na rotina de limpeza
    if (argc == 1)
    {
        printf("Iniciando limpeza do chao\n");

        setIt = grid.notvisited.begin();
        // cout << *setIt << endl;
        // setIt = grid.notvisited.erase(setIt);
        // cout << *setIt << endl;
        // setIt++;
        // cout << *setIt << endl
        //      << endl;
        // cout << endl;

        // set<GridLocation, int>::iterator it;
        // set<GridLocation, int> notvis;
        // notvis.insert(GridLocation{i, j}, 0);
        // it = notvis.begin();
        // cout << *it << endl;
        // for (setIt = grid.notvisited.begin(); setIt != grid.notvisited.end(); setIt++)
        // {
        //     cout << *setIt << endl;
        // }

        //inicializacao
        // for (int j = 0; j < GRID_HEIGHT; j++)
        // {
        //     for (int i = 0; i < GRID_WIDTH; i++)
        //     {
        //         visitado[i][j] = 0;
        //     }
        // }

        visitado[grid_current.x][grid_current.y] = 1;

        // return 0;

        grid_current = {16, 16}; // Posicao inicial do robo na grid

        nextgoal();

        /////////////////////////////////////////////////////////////
        // GridLocation start{16, 16};
        // GridLocation goal{17, 30};

        // auto came_from = breadth_first_search(grid, start, goal);
        // draw_grid(grid, 2, nullptr, &came_from);

        // cout << '\n';
        // path = reconstruct_path(start, goal, came_from);
        // draw_grid(grid, 2, nullptr, nullptr, &path);
        // return 0;
        //////////////////////////////////////////////////////////////

        // srand(time(NULL));
        // goalx = double(rand() % 150) / 10.0;
        // goaly = double(rand() % 150) / 10.0;

        // Printa o GOAL no terminal
        // ROS_INFO("GOAL aleatorio: x=%lf, y=%lf", goalx, goaly);
    }
    else
    {
        if (argc == 4) // Recebe goal como parametro. Nao inicia rotina de limpeza.
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

    nextstep();

    // grid_goal = *grid.notvisited.begin();

    ROS_WARN("%d %d", grid_goal.x, grid_goal.y);
    // return 0;

    // Inicializa goal local com valores do goal global
    // localx = goalx;
    // localy = goaly;

    // VER NECESSIDADE DISSO AQUI
    goalx = map2pos(path[step].x);
    goaly = map2pos(path[step].y);

    // localx = map2pos(path[step].x);
    // localy = map2pos(path[step].y);
    // ROS_INFO("%d", step);
    // step++;
    // ROS_INFO("%d", step);

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

        // Define valores na grid
        grid_current = {int(robox * 2), int(roboy * 2)}; // Posicao atual na grid
        // GridLocation grid_goal{int(goalx * 2), int(goaly * 2)};    // Goal global na grid
        grid_local = {int(localx * 2), int(localx * 2)}; // Goal local na grid

        // Distancia do robo ate o goal global
        dist = sqrt((localx - robox) * (localx - robox) + (localy - roboy) * (localy - roboy));

        // Relacoes entre robo e goal local
        difx = localx - robox;                      // Diferenca entre posicao x do goal local e do robo
        dify = localy - roboy;                      // Diferenca entre posicao y do goal local e do robo
        alfa = atan2(dify, difx);                   // Angulo alfa pode ser calculado com a tangente
        erro_ang = calcula_erro_ang();              // Calcula erro angular
        erro_lin = sqrt(difx * difx + dify * dify); // Calcula erro linear

        visitado[grid_current.x][grid_current.y] = 1; // Marca a posicao atual do mapa como visitada

        // if (grid.notvisited.find(grid_current) != grid.notvisited.end())
        // {
        //     ROS_WARN("Encontrado");
        // }

        // grid.notvisited.erase(grid_current); // Remove os locais já visitados

        // if (grid.notvisited.find(grid_current) == grid.notvisited.end()){
        //     ROS_INFO("Removido com sucesso");
        // }

        // Testing code //
        // grid_local = path[step];
        // ROS_INFO("%lg %lg", robox, roboy);

        // for (size_t i = 0; i < path.size(); i++)
        // {
        //     cout << path[i].x << " " << path[i].y << "\n";
        // }

        // cout << grid.notvisited.size() << '\n';

        // Estado que cuida do movimento ate o goal
        if (estado == 0)
        {
            movetogoal(localx, localy, robox, roboy);
        }

        // Estado que faz o contorno de obstaculos
        // if (estado == 1)
        // {

        // }

        // Estado final, encerra o nodo
        if (estado == 2)
        {
            ROS_WARN("O robo chegou ao GOAL local!");
            ROS_INFO("Goal antigo: %lg %lg", localx, localy);
            ROS_INFO("xy: %lg %lg", robox, roboy);
            if (step == path.size())
            {
                step = 0;
                nextgoal();
            }

            if (grid.notvisited.empty())
            {
                // ENCERRAR ROS - VISITOU TODOS OS LOCAIS POSSIVEIS
                ROS_WARN("Limpeza concluída! Todos os locais possíveis foram visitados.");
                return 0;
            }

            nextstep();
            // localx = map2pos(path[step].x);
            // localy = map2pos(path[step].y);
            // step++;
            ROS_INFO("Novo goal local: %lg %lg\n", localx, localy);

            estado = 0;

            // return 0;
        }

        // ROS_INFO("v1=%lg v2=%lg", v1, v2);
        // cout << '\n';

        // Envia Sinal de Velocidade
        speed_create.linear.x = v1;
        speed_create.angular.z = v2;
        vel_pub.publish(speed_create);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}