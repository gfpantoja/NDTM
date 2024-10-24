#pragma warning (disable: 4786)

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <random>
#include <ilcplex/ilocplex.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cstdlib>

const double M_PI_270 = M_PI + M_PI_2;

ILOSTLBEGIN

using namespace std;
using namespace std::chrono;

// tipos de datos

typedef IloArray<IloNumVarArray> IloNumVarArray2;
typedef IloArray<IloArray<IloNumVarArray>> IloNumVarArray3;
typedef IloArray<IloArray<IloArray<IloNumVarArray>>> IloNumVarArray4;
typedef IloArray<IloArray<IloArray<IloArray<IloNumVarArray>>>> IloNumVarArray5;
typedef IloArray<IloArray<IloArray<IloArray<IloArray<IloNumVarArray>>>>> IloNumVarArray6;
typedef IloArray<IloArray<IloArray<IloArray<IloArray<IloArray<IloNumVarArray>>>>>> IloNumVarArray7;
typedef IloArray<IloArray<IloArray<IloArray<IloArray<IloArray<IloArray<IloNumVarArray>>>>>>> IloNumVarArray8;
typedef IloArray<IloBoolVarArray> IloBoolVarArray2;
typedef IloArray<IloArray<IloBoolVarArray>> IloBoolVarArray3;
typedef IloArray<IloArray<IloArray<IloBoolVarArray>>> IloBoolVarArray4;
typedef IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>> IloBoolVarArray5;
typedef IloArray<IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>>> IloBoolVarArray6;
typedef IloArray<IloArray<IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>>>> IloBoolVarArray7;
typedef IloArray<IloArray<IloArray<IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>>>>> IloBoolVarArray8;
typedef IloArray<IloArray<IloArray<IloArray<IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>>>>>> IloBoolVarArray9;
typedef IloArray<IloArray<IloArray<IloArray<IloArray<IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>>>>>>> IloBoolVarArray10;

// VARIABLES

const double errorG = 1e-6;
double Ly = 0.0f;
double minLx = -1.0f;
double maxLx = 0.0f;
int maxTime = 3600;
int nThreads = 8;
string fileName = "";
time_point<std::chrono::system_clock> timeIni, timeFin;
duration<double> duracion;

// CLASES

// clases auxiliares

class Vec2 {
public:

    // Par�metros

    double x, y;

    // Constructor

    Vec2() {
        x = 0;
        y = 0;
    }
    Vec2(double const& x1, double const& y1) {
        x = x1;
        y = y1;
    }

    // M�todos

    void div(double const& numDiv) {
        x /= numDiv;
        y /= numDiv;
    }

    // Operadores

    bool operator==(const Vec2 otro) {
        return abs(otro.x - x) < errorG && abs(otro.y - y) < errorG;
    }
};
class Vec3 {
public:

    // Par�metros

    double x, y, z;
    double ax, ay, bx, by;

    // Constructores

    Vec3() {
    }
    Vec3(double const& x1, double const& y1, double const& z1) {
        x = x1;
        y = y1;
        z = z1;
        ax = 0;
        ay = 0;
        bx = 0;
        by = 0;
    }
    Vec3(Vec2 const& a, Vec2 const& b) {
        ax = b.x;
        ay = b.y;
        bx = a.x;
        by = a.y;
        double A = b.y - a.y;
        double B = a.x - b.x;
        double nAB = sqrt(A * A + B * B);
        if (nAB > errorG) {
            x = A / nAB;
            y = B / nAB;
            z = -a.x * x - a.y * y;
        }
        else
        {
            x = 0;
            y = 0;
            z = 0;
        }
    }

    // M�todos

    void div(double const& numDiv) {
        x /= numDiv;
        y /= numDiv;
        z /= numDiv;
    }
    void neg() {
        x = -x;
        y = -y;
        z = -z;
    }
    void cambiarAB() {
        double temp = ax;
        ax = bx;
        bx = temp;
        temp = ay;
        ay = by;
        by = temp;
    }
    double funD(Vec2 const& punto) {
        return (ax - bx) * (ay - punto.y) - (ay - by) * (ax - punto.x);
    }
};

// Funci�n auxiliar de distancia

double distancia(Vec3 const& edge, Vec2 const& punto) {
    return edge.x * punto.x + edge.y * punto.y + edge.z;
}

// Clases de las piezas

class Parte {
public:

    // Par�metros

    int id;
    Vec2 centro, AABB1, AABB2;
    vector<Vec2> vertices;
    vector<Vec3> lineas;
    vector<vector<vector<vector<double>>>> C2;
    vector<vector<vector<vector<double>>>> M;
    vector<vector<vector<vector<double>>>> M0;
    vector<vector<vector<vector<bool>>>> PuedeSeparar;

    // Constructor

    Parte() {
        centro = Vec2(0, 0);
        vertices = vector<Vec2>(0);
        lineas = vector<Vec3>(0);
        AABB1 = Vec2(0, 0);
        AABB2 = Vec2(0, 0);
        C2 = vector<vector<vector<vector<double>>>>(0);
        M = vector<vector<vector<vector<double>>>>(0);
        PuedeSeparar = vector<vector<vector<vector<bool>>>>(0);
        id = 0;
    }
    Parte(int const& p_id, vector<Vec2> const& p_vertices) {
        id = p_id;
        vertices = p_vertices;

        // Ordenar los v�rtices de forma contigua e ir a�adiendo los bordes

        for (vector<Vec2>::iterator a = vertices.begin(); a < vertices.end() - 1; ++a) {
            for (vector<Vec2>::iterator b = a + 1; b < vertices.end(); ++b) {
                Vec3 borde = Vec3(*a, *b);
                if (abs(borde.x) > errorG || abs(borde.y) > errorG) {
                    bool esBorde = true;
                    for (vector<Vec2>::iterator c = vertices.begin(); c < vertices.end(); ++c) {
                        if (a != c && b != c) {
                            if (distancia(borde, *c) < 0.0f) {
                                esBorde = false;
                                break;
                            }
                        }
                    }
                    if (esBorde) {
                        if (b != a + 1) {
                            iter_swap(a + 1, b);
                        }
                        lineas.push_back(borde);
                        break;
                    }
                }
            }
        }
        lineas.push_back(Vec3(vertices.back(), vertices.front()));

        // Centro y extremos

        centro = Vec2(vertices.front().x, vertices.front().y);
        AABB1 = centro;
        AABB2 = centro;
        for (vector<Vec2>::iterator v = vertices.begin() + 1; v < vertices.end(); ++v) {
            centro.x += (*v).x;
            centro.y += (*v).y;
            AABB1.x = min(AABB1.x, (*v).x);
            AABB1.y = min(AABB1.y, (*v).y);
            AABB2.x = max(AABB2.x, (*v).x);
            AABB2.y = max(AABB2.y, (*v).y);
        }
        centro.div(vertices.size());

        // Orientar las l�neas

        for (vector<Vec3>::iterator l_it = lineas.begin(); l_it < lineas.end(); ++l_it) {
            if (distancia((*l_it), centro) < 0) {
                (*l_it).neg();
            }
            if ((*l_it).funD(centro) < 0) {
                (*l_it).cambiarAB();
            }
        }

        // Par�metros

        C2 = vector<vector<vector<vector<double>>>>(0);
        M = vector<vector<vector<vector<double>>>>(0);
        PuedeSeparar = vector<vector<vector<vector<bool>>>>(0);
    }

    // M�todos

    void ParteGirada(double const& angulo) {
        double c = cos(angulo);
        double s = sin(angulo);

        // V�rtices

        centro = Vec2(centro.x * c - centro.y * s, centro.x * s + centro.y * c);
        for (vector<Vec2>::iterator v = vertices.begin(); v < vertices.end(); ++v) {
            *v = Vec2((*v).x * c - (*v).y * s, (*v).x * s + (*v).y * c);
        }

        // L�neas

        lineas.clear();
        for (vector<Vec2>::iterator v = vertices.begin(); v < vertices.end() - 1; ++v) {
            lineas.push_back(Vec3(*v, *(v + 1)));
            if (distancia(lineas.back(), centro) < 0) {
                lineas.back().neg();
            }
            if (lineas.back().funD(centro) < 0) {
                lineas.back().cambiarAB();
            }
        }
        lineas.push_back(Vec3(vertices.back(), vertices.front()));
        if (distancia(lineas.back(), centro) < 0) {
            lineas.back().neg();
        }
        if (lineas.back().funD(centro) < 0) {
            lineas.back().cambiarAB();
        }

        // Bounding Box

        AABB1 = Vec2(vertices.front().x, vertices.front().y);
        AABB2 = AABB1;
        for (vector<Vec2>::iterator v = vertices.begin() + 1; v < vertices.end(); ++v) {
            AABB1.x = min(AABB1.x, (*v).x);
            AABB1.y = min(AABB1.y, (*v).y);
            AABB2.x = max(AABB2.x, (*v).x);
            AABB2.y = max(AABB2.y, (*v).y);
        }
    }
};
class PiezaG {
public:

    // Par�metros

    int id, idTipo, aux;
    double anguloRot, metrica, xy;
    Vec2 dim, dmin, dmax;
    vector<Vec2> vertices;
    vector<Vec2> verticesMod;
    vector<Parte> partes;

    // Constructor

    PiezaG(double const& p_anguloRot, vector<Parte> const& p_partes, int const& p_idTipo) {
        idTipo = p_idTipo;
        anguloRot = p_anguloRot;
        partes = p_partes;
        if (anguloRot != 0.0f) {
            for (vector<Parte>::iterator p_it = partes.begin(); p_it < partes.end(); ++p_it) {
                (*p_it).ParteGirada(p_anguloRot);
            }
        }
        vertices = vector<Vec2>(partes.front().vertices);
        for (vector<Parte>::iterator p_it = partes.begin() + 1; p_it < partes.end(); ++p_it) {
            vertices.insert(vertices.end(), (*p_it).vertices.begin(), (*p_it).vertices.end());
        }
        sort(vertices.begin(), vertices.end(), [](Vec2 v1, Vec2 v2)->bool {return v1.y < v2.y; });
        stable_sort(vertices.begin(), vertices.end(), [](Vec2 v1, Vec2 v2)->bool {return v1.x < v2.x; });
        vertices.resize(distance(vertices.begin(), unique(vertices.begin(), vertices.end())));
        dmin = Vec2(0.0f, 0.0f);
        dmax = Vec2(0.0f, 0.0f);
        for (vector<Vec2>::iterator v_it = vertices.begin(); v_it < vertices.end(); ++v_it) {
            dmin.x = min(dmin.x, (*v_it).x);
            dmin.y = min(dmin.y, (*v_it).y);
            dmax.x = max(dmax.x, (*v_it).x);
            dmax.y = max(dmax.y, (*v_it).y);
        }
        dmin = Vec2(abs(dmin.x), abs(dmin.y));
        dim = Vec2(dmax.x + dmin.x, dmax.y + dmin.y);
        verticesMod = vector<Vec2>(vertices);
        for (vector<Vec2>::iterator v_it = verticesMod.begin(); v_it < verticesMod.end(); ++v_it) {
            (*v_it).x -= dmax.x;
            (*v_it).y -= dmax.y;
        }
        metrica = -1;
        xy = dim.x + dim.y;
        aux = 0;
    }
};
class TipoPieza {
public:

    // Par�metros

    int cantidad0, cantidad, id, dx0, dy0;
    double miny, maxy, minx;
    vector<Parte> partes0;
    vector<PiezaG> piezas;

    // Constructor

    TipoPieza(const int& p_id, const int& p_cantidad, vector<double>& p_rotaciones, vector<Vec2>& p_vertices, vector<vector<int>>& p_partes) {
        id = p_id;
        cantidad0 = p_cantidad;
        cantidad = p_cantidad;

        // Se hace que el primer v�rtice sea (0,0)

        if ((p_vertices.front().x != 0 || p_vertices.front().y != 0)) {
            double restax = p_vertices.front().x;
            double restay = p_vertices.front().y;
            for (vector<Vec2>::iterator v_it = p_vertices.begin(); v_it < p_vertices.end(); ++v_it) {
                (*v_it).x -= restax;
                (*v_it).y -= restay;
            }
        }

        // Se hacen las partes con los v�rtices que vienen de entrada

        int j = 0;
        for (vector<vector<int>>::iterator p2_it = p_partes.begin(); p2_it < p_partes.end(); ++p2_it) {
            vector<Vec2> vert;
            vert.reserve(p_vertices.size());
            for (vector<int>::iterator p1_it = (*p2_it).begin(); p1_it < (*p2_it).end(); ++p1_it) {
                vert.push_back(p_vertices[*p1_it]);
            }
            partes0.push_back(Parte(j, vert));
            ++j;
        }

        // Se forman las piezas

        piezas.reserve(p_rotaciones.size());
        for (vector<double>::iterator r_it = p_rotaciones.begin(); r_it < p_rotaciones.end(); ++r_it) {
            piezas.push_back(PiezaG(*r_it, partes0, id));
        }

        // Se eliminan las piezas que no alcanzan en el strip

        for (int i = piezas.size() - 1; i >= 0; --i) {
            if (piezas[i].dim.y > Ly) {
                piezas.erase(piezas.begin() + i);
            }
        }

        // Se eliminan las piezas iguales seg�n la posici�n de los v�rtices

        for (int i = 0; i < piezas.size() - 1; ++i) {
            for (int j = piezas.size() - 1; j > i; --j) {
                bool piezaIgual = true;
                for (vector<Vec2>::iterator v_it = piezas[i].verticesMod.begin(); v_it < piezas[i].verticesMod.end(); ++v_it) {
                    bool puntoDiferente = true;
                    for (vector<Vec2>::iterator v_jt = piezas[j].verticesMod.begin(); v_jt < piezas[j].verticesMod.end(); ++v_jt) {
                        if ((*v_jt).x + errorG < (*v_it).x) {
                            continue;
                        }
                        else if (abs((*v_jt).x - (*v_it).x) < errorG) {
                            if (abs((*v_jt).y - (*v_it).y) < errorG) {
                                puntoDiferente = false;
                                break;
                            }
                        }
                        else {
                            break;
                        }
                    }
                    if (puntoDiferente) {
                        piezaIgual = false;
                        break;
                    }
                }
                if (piezaIgual) {
                    piezas.erase(piezas.begin() + j);
                }
            }
        }

        // Se enumeran las piezas de cada tipo

        int i = 0;
        for (vector<PiezaG>::iterator p_it = piezas.begin(); p_it < piezas.end(); ++p_it) {
            (*p_it).id = i;
            ++i;
        }

        // Se actualizan las cantidades de los tipos de pieza que no tienen piezas

        if (piezas.size() == 0) {
            cantidad = 0;
        }

        // Se determina miny

        miny = piezas.front().dim.y;
        maxy = piezas.front().dim.y;
        minx = piezas.front().dim.x;
        for (vector<PiezaG>::iterator p_it = piezas.begin() + 1; p_it < piezas.end(); ++p_it) {
            if (miny > (*p_it).dim.y) {
                miny = (*p_it).dim.y;
            }
            else if (maxy < (*p_it).dim.y) {
                maxy = (*p_it).dim.y;
            }
            if (minx > (*p_it).dim.x) {
                minx = (*p_it).dim.x;
            }
        }

        // Se determinan dx0 y dy0

        dx0 = (int)ceil(piezas.front().dim.x);
        dy0 = (int)ceil(piezas.front().dim.y);
    }
};

vector<TipoPieza> tiposPieza;

int totalPiezas = 0;
int totalLineasSeparadoras = 0; // No es del modelo completo
vector<vector<vector<vector<int>>>> nVariables;
double miny0 = 0;
double maxy0 = 0;

// Clases de empaquetamiento

class PiezaEmpacada {
public:

    // Par�metros

    int idUnico;
    PiezaG* pieza;
    Vec2 posicion, AABB1, AABB2;
    bool comprimida;

    // Constructivo

    PiezaEmpacada() {
        pieza = nullptr;
        posicion = Vec2(0.0f, 0.0f);
        AABB1 = Vec2(0.0f, 0.0f);
        AABB2 = Vec2(0.0f, 0.0f);
        comprimida = false;
        idUnico = -1;
    }
    PiezaEmpacada(PiezaG* const& p_pieza, Vec2 const& p_posicion, int const& p_idUnico) {
        pieza = p_pieza;
        posicion = p_posicion;
        AABB1 = Vec2(p_posicion.x - (*p_pieza).dmin.x, p_posicion.y - (*p_pieza).dmin.y);
        AABB2 = Vec2(p_posicion.x + (*p_pieza).dmax.x, p_posicion.y + (*p_pieza).dmax.y);
        comprimida = false;
        idUnico = p_idUnico;
    }
    PiezaEmpacada(PiezaEmpacada const& p_piezaE, Vec2 const& p_posicion) {
        comprimida = p_piezaE.comprimida;
        pieza = p_piezaE.pieza;
        posicion = p_posicion;
        AABB1 = Vec2(p_posicion.x - (*pieza).dmin.x, p_posicion.y - (*pieza).dmin.y);
        AABB2 = Vec2(p_posicion.x + (*pieza).dmax.x, p_posicion.y + (*pieza).dmax.y);
        idUnico = p_piezaE.idUnico;
    }

    // M�todos

    void ActualizarLimites(double const& posx, double const& posy) {
        posicion = Vec2(posx, posy);
        AABB1 = Vec2(posx - (*pieza).dmin.x, posy - (*pieza).dmin.y);
        AABB2 = Vec2(posx + (*pieza).dmax.x, posy + (*pieza).dmax.y);
    }

    // Operadores
    /*
    const bool operator== (PiezaEmpacada const& otra) {
        return (pieza == otra.pieza && posicion.x == otra.posicion.x && posicion.y == otra.posicion.y);
    }
    */
};
class LineaSeparadora {
public:

    // Par�metros

    PiezaEmpacada* pieza1;
    Parte* parte1;
    Vec3* linea1;
    int indLinea1, indTipo2, indPieza2, indParte2;
    double d;

    // Constructivo

    LineaSeparadora() {
        pieza1 = nullptr;
        parte1 = nullptr;
        linea1 = nullptr;
        indLinea1 = -1;
        indTipo2 = -1;
        indPieza2 = -1;
        indParte2 = -1;
        d = -1.0f;
    }
    LineaSeparadora(PiezaEmpacada* const& p_pieza1, Parte* const& p_parte1, Vec3* const& p_linea1, int const& p_indLinea1, int const& p_indTipo2, int const& p_indPieza2, int const& p_indParte2, double const& p_d) {
        pieza1 = p_pieza1;
        parte1 = p_parte1;
        linea1 = p_linea1;
        indLinea1 = p_indLinea1;
        indTipo2 = p_indTipo2;
        indPieza2 = p_indPieza2;
        indParte2 = p_indParte2;
        d = p_d;
    }
};

class Solucion {
public:

    // Par�metros

    int nPiezasColocadas, maxVariables, maxVariables0, totalVariables, nAumentos, id;
    double Lx, miny, maxy;
    vector<PiezaEmpacada> piezasEmpacadas;
    vector<PiezaEmpacada*> piezasEmpacadasTemp;
    vector<vector<vector<vector<LineaSeparadora>>>> lineasSeparadoras;
    vector<vector<vector<vector<vector<bool>>>>> u, u0;
    vector<vector<int>> lineasSeparadorasAABB; // 0 = N/A, 1 = izquierda, 2 = derecha, 3 = abajo, 4 = arriba
    vector<vector<bool>> parejas;

    // Constructivo

    Solucion() {
        id = 0;
        Lx = 0.0f;
        miny = miny0;
        maxy = maxy0;
        if (abs(maxy - Ly) < errorG) {
            --maxy;
        }
        piezasEmpacadas = vector<PiezaEmpacada>(0);
        piezasEmpacadas.reserve(totalPiezas);
        piezasEmpacadasTemp = vector<PiezaEmpacada*>(0);
        piezasEmpacadasTemp.reserve(totalPiezas);

        // Reestablcer cantidad de tipos de puezas

        for (vector<TipoPieza>::iterator t_it = tiposPieza.begin(); t_it < tiposPieza.end(); ++t_it) {
            (*t_it).cantidad = (*t_it).cantidad0;
        }
        nPiezasColocadas = 0;
        lineasSeparadoras = vector<vector<vector<vector<LineaSeparadora>>>>(0);
        u0 = vector<vector<vector<vector<vector<bool>>>>>(0);
        u = vector<vector<vector<vector<vector<bool>>>>>(0);
        lineasSeparadorasAABB = vector<vector<int>>(0);
        maxVariables0 = 0;
        maxVariables = 0;
        totalVariables = 0;
        nAumentos = 0;
    }
    Solucion(int& p_id) {
        id = p_id;
        ++p_id;
        Lx = 0.0f;
        miny = miny0;
        maxy = maxy0;
        if (abs(maxy - Ly) < errorG) {
            --maxy;
        }
        piezasEmpacadas = vector<PiezaEmpacada>(0);
        piezasEmpacadas.reserve(totalPiezas);
        piezasEmpacadasTemp = vector<PiezaEmpacada*>(0);
        piezasEmpacadasTemp.reserve(totalPiezas);

        // Reestablcer cantidad de tipos de puezas

        for (vector<TipoPieza>::iterator t_it = tiposPieza.begin(); t_it < tiposPieza.end(); ++t_it) {
            (*t_it).cantidad = (*t_it).cantidad0;
        }
        nPiezasColocadas = 0;
        lineasSeparadoras = vector<vector<vector<vector<LineaSeparadora>>>>(0);
        u0 = vector<vector<vector<vector<vector<bool>>>>>(0);
        u = vector<vector<vector<vector<vector<bool>>>>>(0);
        lineasSeparadorasAABB = vector<vector<int>>(0);
        maxVariables0 = 0;
        maxVariables = 0;
        totalVariables = 0;
        nAumentos = 0;
    }
};

// FUNCIONES

void UpdateTime() {
    timeFin = std::chrono::system_clock::now();
    duracion = timeFin - timeIni;
}

// Lectura de datos

void ReadData() {

    // Variables

    int nI = 0;
    int nPi = 0;
    int nPa = 0;
    int nV = 0;
    int nR = 0;
    int fila = 0;
    vector<double> rotaciones;
    vector<Vec2> vertices;
    vector<vector<int>> partes;
    vector<int> partes1;

    int idUnico = 0;

    // Lectura de datos

    ifstream file("Instances/" + fileName + ".txt"); string line;
    while (getline(file, line, '\n')) {
        stringstream iss(line); string val;
        if (fila == 0) {
            getline(iss, val, ' ');
            stringstream convertor(val);
            convertor >> Ly;
            getline(iss, val, ' ');
            stringstream convertor1(val);
            convertor1 >> nI;
            tiposPieza.reserve(nI);
            nI = 0;
        }
        else if (fila == 1) {
            getline(iss, val, ' ');
            stringstream convertor(val);
            convertor >> nPi;
            getline(iss, val, ' ');
            stringstream convertor1(val);
            convertor1 >> nPa;
            getline(iss, val, ' ');
            stringstream convertor2(val);
            convertor2 >> nV;
            getline(iss, val, ' ');
            stringstream convertor3(val);
            convertor3 >> nR;
            partes.clear();
            partes.reserve(nPa);
            vertices.clear();
            vertices.reserve(nV);
            rotaciones.clear();
            rotaciones.reserve(nR);
        }
        else if (fila == 2) {
            while (getline(iss, val, ' ')) {
                stringstream convertor(val);
                double faux;
                convertor >> faux;
                rotaciones.push_back(faux * M_PI / 180);
            }
        }
        else if (fila < 3 + nPa) {
            partes1.clear();
            partes1.reserve(nV);
            while (getline(iss, val, ' ')) {
                stringstream convertor(val);
                int iaux;
                convertor >> iaux;
                partes1.push_back(iaux);
            }
            partes.push_back(partes1);
        }
        else if (fila < 2 + nPa + nV) {
            double f1, f2;
            getline(iss, val, ' ');
            stringstream convertor(val);
            convertor >> f1;
            getline(iss, val, ' ');
            stringstream convertor1(val);
            convertor1 >> f2;
            vertices.push_back(Vec2(f1, f2));
        }
        else {
            double f1, f2;
            getline(iss, val, ' ');
            stringstream convertor(val);
            convertor >> f1;
            getline(iss, val, ' ');
            stringstream convertor1(val);
            convertor1 >> f2;
            vertices.push_back(Vec2(f1, f2));

            // Creaci�n de pieza
            tiposPieza.push_back(TipoPieza(nI, nPi, rotaciones, vertices, partes));
            ++nI;
            fila = 0;
        }
        ++fila;
    }
    file.close();
}
void WriteSolution(Solucion& sol, double const& gap = 0) {
    ofstream outFile;
    outFile.open("Solutions/Sol_" + fileName + ".txt", ios::out);

    outFile << "Time: " << duracion.count() << endl;
    if (gap == 0) {
        outFile << "Lx: " << sol.Lx << endl;
    }
    else {
        outFile << "Lx: " << sol.Lx << " " << gap << endl;
    }
    outFile << "Ly: " << Ly << endl;
    outFile << "Id TipoPieza x y �ngulo" << endl;
    //sort(sol.piezasEmpacadas.begin(), sol.piezasEmpacadas.end(), [](PiezaEmpacada p1, PiezaEmpacada p2)->bool {return (*p1.pieza).idTipo < (*p2.pieza).idTipo;});
    int i = 0;
    for (vector<PiezaEmpacada>::iterator p_it = sol.piezasEmpacadas.begin(); p_it < sol.piezasEmpacadas.end(); ++p_it) {
        outFile << i << " " << (*(*p_it).pieza).idTipo << " " << (*p_it).posicion.x << " " << (*p_it).posicion.y << " " << (*(*p_it).pieza).anguloRot * 180 / M_PI << endl;
        ++i;
    }
    outFile.close();
}

// Par�metros de optimizaci�n

void AuxiliarCM(vector<double>& C21, vector<double>& M1, vector<Vec3>::iterator const& l1_it, vector<Parte>::iterator const& pp2_it, vector<Parte>::iterator const& pp1_it, int const& linea, vector<PiezaG>::iterator const& p1_it, vector<PiezaG>::iterator const& p2_it) {
    double maxVal = ((*l1_it).ax - (*l1_it).bx) * ((*l1_it).ay - (*pp2_it).vertices.front().y) - ((*l1_it).ay - (*l1_it).by) * ((*l1_it).ax - (*pp2_it).vertices.front().x);
    for (vector<Vec2>::iterator v_it = (*pp2_it).vertices.begin() + 1; v_it < (*pp2_it).vertices.end(); ++v_it) {
        double otroVal = ((*l1_it).ax - (*l1_it).bx) * ((*l1_it).ay - (*v_it).y) - ((*l1_it).ay - (*l1_it).by) * ((*l1_it).ax - (*v_it).x);
        if (maxVal < otroVal) {
            maxVal = otroVal;
        }
    }
    C21.push_back(maxVal);
    M1.push_back(maxVal + abs((*l1_it).ax - (*l1_it).bx) * Ly);
}
void CalcularParametros() {

    // Mis par�metros

    maxLx = 0;
    totalPiezas = 0;
    miny0 = Ly;
    for (vector<TipoPieza>::iterator t_it = tiposPieza.begin(); t_it < tiposPieza.end(); ++t_it) {
        maxLx += (double)(*t_it).cantidad0 * (*t_it).minx;
        totalPiezas += (*t_it).cantidad0;
        miny0 = min(miny0, (*t_it).miny);
        maxy0 = max(maxy0, (*t_it).maxy);
    }
    //cout << "MaxL = " << maxLx << endl;
    vector<vector<int>> nVariablesTipos(tiposPieza.size(), vector<int>(tiposPieza.size(), 0));
    totalLineasSeparadoras = 0;
    int i = 0;
    for (vector<TipoPieza>::iterator t1_it = tiposPieza.begin(); t1_it < tiposPieza.end(); ++t1_it, ++i) {
        if ((*t1_it).cantidad0 > 1) {
            totalLineasSeparadoras += (((*t1_it).cantidad0 - 1) * (*t1_it).cantidad0) / 2 * (*t1_it).partes0.size() * (*t1_it).partes0.size();
            int nLineas = 0;
            for (vector<Parte>::iterator pp_it = (*t1_it).partes0.begin(); pp_it < (*t1_it).partes0.end(); ++pp_it) {
                nLineas += (*pp_it).lineas.size();
            }
            nVariablesTipos[i][i] = nLineas * (*t1_it).partes0.size() * 2;
        }
        int nLineas1 = 0;
        for (vector<Parte>::iterator pp_it = (*t1_it).partes0.begin(); pp_it < (*t1_it).partes0.end(); ++pp_it) {
            nLineas1 += (*pp_it).lineas.size();
        }
        int j = i + 1;
        for (vector<TipoPieza>::iterator t2_it = t1_it + 1; t2_it < tiposPieza.end(); ++t2_it, ++j) {
            totalLineasSeparadoras += (*t1_it).partes0.size() * (*t1_it).cantidad0 * (*t2_it).partes0.size() * (*t2_it).cantidad0;
            int nLineas2 = 0;
            for (vector<Parte>::iterator pp_it = (*t2_it).partes0.begin(); pp_it < (*t2_it).partes0.end(); ++pp_it) {
                nLineas2 += (*pp_it).lineas.size();
            }
            int val = nLineas1 * (*t2_it).partes0.size() + nLineas2 * (*t1_it).partes0.size();
            nVariablesTipos[i][j] = val;
            nVariablesTipos[j][i] = val;
        }
    }
    nVariables = vector<vector<vector<vector<int>>>>(tiposPieza.size());
    i = 0;
    for (vector<TipoPieza>::iterator t1_it = tiposPieza.begin(); t1_it < tiposPieza.end(); ++t1_it, ++i) {
        vector<vector<vector<int>>> nVariables3((*t1_it).piezas.size(), vector<vector<int>>(tiposPieza.size()));
        int j = 0;
        for (vector<PiezaG>::iterator p1_it = (*t1_it).piezas.begin(); p1_it < (*t1_it).piezas.end(); ++p1_it, ++j) {
            int ii = 0;
            for (vector<TipoPieza>::iterator t2_it = tiposPieza.begin(); t2_it < tiposPieza.end(); ++t2_it, ++ii) {
                vector<int> nVariables1((*t2_it).piezas.size(), nVariablesTipos[i][ii]);
                nVariables3[j][ii] = nVariables1;
            }
        }
        nVariables[i] = nVariables3;
    }
    i = 0;
    for (vector<TipoPieza>::iterator t1_it = tiposPieza.begin(); t1_it < tiposPieza.end(); ++t1_it, ++i) {
        int j = 0;
        for (vector<PiezaG>::iterator p1_it = (*t1_it).piezas.begin(); p1_it < (*t1_it).piezas.end(); ++p1_it, ++j) {
            for (vector<Parte>::iterator pp1_it = (*p1_it).partes.begin(); pp1_it < (*p1_it).partes.end(); ++pp1_it) {
                (*pp1_it).C2.reserve((*pp1_it).lineas.size());
                (*pp1_it).M0.reserve((*pp1_it).lineas.size());
                (*pp1_it).PuedeSeparar.reserve((*pp1_it).lineas.size());
                int linea = 0;
                for (vector<Vec3>::iterator l1_it = (*pp1_it).lineas.begin(); l1_it < (*pp1_it).lineas.end(); ++l1_it) {
                    vector<vector<vector<double>>> C23;
                    C23.reserve(tiposPieza.size());
                    vector<vector<vector<double>>> M3;
                    M3.reserve(tiposPieza.size());
                    vector<vector<vector<bool>>> PuedeSeparar3;
                    PuedeSeparar3.reserve(tiposPieza.size());
                    int ii = 0;
                    for (vector<TipoPieza>::iterator t2_it = tiposPieza.begin(); t2_it < tiposPieza.end(); ++t2_it) {
                        vector<vector<double>> C22;
                        C22.reserve((*t2_it).piezas.size());
                        vector<vector<double>> M2;
                        M2.reserve((*t2_it).piezas.size());
                        vector<vector<bool>> PuedeSeparar2;
                        PuedeSeparar2.reserve((*t2_it).piezas.size());
                        if (t1_it < t2_it) { // Se descartan lineas paralelas
                            int jj = 0;
                            for (vector<PiezaG>::iterator p2_it = (*t2_it).piezas.begin(); p2_it < (*t2_it).piezas.end(); ++p2_it) {
                                vector<double> C21;
                                C21.reserve((*p2_it).partes.size());
                                vector<double> M1;
                                M1.reserve((*p2_it).partes.size());
                                vector<bool> PuedeSeparar1;
                                PuedeSeparar1.reserve((*p2_it).partes.size());
                                for (vector<Parte>::iterator pp2_it = (*p2_it).partes.begin(); pp2_it < (*p2_it).partes.end(); ++pp2_it) {

                                    // Determinar si la parte tiene una l�nea paralela en el sentido contrario a la que se considera

                                    bool hayLineaParalelaContraria = false;
                                    for (vector<Vec3>::iterator l2_it = (*pp2_it).lineas.begin(); l2_it < (*pp2_it).lineas.end(); ++l2_it) {
                                        if (abs((*l1_it).x + (*l2_it).x) < errorG) {
                                            if (abs((*l1_it).y + (*l2_it).y) < errorG) {
                                                hayLineaParalelaContraria = true;
                                                break;
                                            }
                                        }
                                    }
                                    if (hayLineaParalelaContraria) {
                                        C21.push_back(0);
                                        M1.push_back(0);
                                        PuedeSeparar1.push_back(false);
                                        --nVariables[i][j][ii][jj];
                                        --nVariables[ii][jj][i][j];
                                    }
                                    else {
                                        AuxiliarCM(C21, M1, l1_it, pp2_it, pp1_it, linea, p1_it, p2_it);
                                        PuedeSeparar1.push_back(true);
                                    }
                                }
                                C22.push_back(C21);
                                M2.push_back(M1);
                                PuedeSeparar2.push_back(PuedeSeparar1);
                                ++jj;
                            }
                        }
                        else if (t1_it > t2_it) { // Se consideran todas las l�neas
                            for (vector<PiezaG>::iterator p2_it = (*t2_it).piezas.begin(); p2_it < (*t2_it).piezas.end(); ++p2_it) {
                                vector<double> C21;
                                C21.reserve((*p2_it).partes.size());
                                vector<double> M1;
                                M1.reserve((*p2_it).partes.size());
                                vector<bool> PuedeSeparar1((*p2_it).partes.size(), true);
                                for (vector<Parte>::iterator pp2_it = (*p2_it).partes.begin(); pp2_it < (*p2_it).partes.end(); ++pp2_it) {
                                    AuxiliarCM(C21, M1, l1_it, pp2_it, pp1_it, linea, p1_it, p2_it);
                                }
                                C22.push_back(C21);
                                M2.push_back(M1);
                                PuedeSeparar2.push_back(PuedeSeparar1);
                            }
                        }
                        else if ((*t1_it).cantidad0 > 1) { // Mismo tipo de pieza
                            int jj = 0;
                            for (vector<PiezaG>::iterator p2_it = (*t2_it).piezas.begin(); p2_it < (*t2_it).piezas.end(); ++p2_it) {
                                vector<double> C21;
                                C21.reserve((*p2_it).partes.size());
                                vector<double> M1;
                                M1.reserve((*p2_it).partes.size());
                                vector<bool> PuedeSeparar1;
                                PuedeSeparar1.reserve((*p2_it).partes.size());
                                if (p1_it < p2_it) { // se descartan l�neas paralelas
                                    for (vector<Parte>::iterator pp2_it = (*p2_it).partes.begin(); pp2_it < (*p2_it).partes.end(); ++pp2_it) {

                                        // Determinar si la parte tiene una l�nea paralela en el sentido contrario a la que se considera

                                        bool hayLineaParalelaContraria = false;
                                        for (vector<Vec3>::iterator l2_it = (*pp2_it).lineas.begin(); l2_it < (*pp2_it).lineas.end(); ++l2_it) {
                                            if (abs((*l1_it).x + (*l2_it).x) < errorG) {
                                                if (abs((*l1_it).y + (*l2_it).y) < errorG) {
                                                    hayLineaParalelaContraria = true;
                                                    break;
                                                }
                                            }
                                        }
                                        if (hayLineaParalelaContraria) {
                                            C21.push_back(0);
                                            M1.push_back(0);
                                            PuedeSeparar1.push_back(false);
                                            --nVariables[i][j][ii][jj];
                                        }
                                        else {
                                            AuxiliarCM(C21, M1, l1_it, pp2_it, pp1_it, linea, p1_it, p2_it);
                                            PuedeSeparar1.push_back(true);
                                        }
                                    }
                                }
                                else if (p1_it > p2_it) { // Se consideran todas las l�neas
                                    PuedeSeparar1 = vector<bool>((*p2_it).partes.size(), true);
                                    for (vector<Parte>::iterator pp2_it = (*p2_it).partes.begin(); pp2_it < (*p2_it).partes.end(); ++pp2_it) {
                                        AuxiliarCM(C21, M1, l1_it, pp2_it, pp1_it, linea, p1_it, p2_it);
                                    }
                                }
                                else {
                                    for (vector<Parte>::iterator pp2_it = (*p2_it).partes.begin(); pp2_it < (*p2_it).partes.end(); ++pp2_it) {
                                        if (pp1_it < pp2_it) { // Se descartan l�neas paralelas

                                            // Determinar si la parte tiene una l�nea paralela en el sentido contrario a la que se considera

                                            bool hayLineaParalelaContraria = false;
                                            for (vector<Vec3>::iterator l2_it = (*pp2_it).lineas.begin(); l2_it < (*pp2_it).lineas.end(); ++l2_it) {
                                                if (abs((*l1_it).x + (*l2_it).x) < errorG) {
                                                    if (abs((*l1_it).y + (*l2_it).y) < errorG) {
                                                        hayLineaParalelaContraria = true;
                                                        break;
                                                    }
                                                }
                                            }
                                            if (hayLineaParalelaContraria) {
                                                C21.push_back(0);
                                                M1.push_back(0);
                                                PuedeSeparar1.push_back(false);
                                                --nVariables[i][j][ii][jj];
                                            }
                                            else {
                                                AuxiliarCM(C21, M1, l1_it, pp2_it, pp1_it, linea, p1_it, p2_it);
                                                PuedeSeparar1.push_back(true);
                                            }
                                        }
                                        else { // Se consideran todas las l�neas
                                            AuxiliarCM(C21, M1, l1_it, pp2_it, pp1_it, linea, p1_it, p2_it);
                                            PuedeSeparar1.push_back(true);
                                        }
                                    }
                                }
                                C22.push_back(C21);
                                M2.push_back(M1);
                                PuedeSeparar2.push_back(PuedeSeparar1);
                                ++jj;
                            }
                        }
                        C23.push_back(C22);
                        M3.push_back(M2);
                        PuedeSeparar3.push_back(PuedeSeparar2);
                        ++ii;
                    }
                    (*pp1_it).C2.push_back(C23);
                    (*pp1_it).M0.push_back(M3);
                    (*pp1_it).PuedeSeparar.push_back(PuedeSeparar3);
                    ++linea;
                }
            }
        }
    }
}
void RecalcularM(double const& miLx = maxLx) {
    for (vector<TipoPieza>::iterator t1_it = tiposPieza.begin(); t1_it < tiposPieza.end(); ++t1_it) {
        for (vector<PiezaG>::iterator p1_it = (*t1_it).piezas.begin(); p1_it < (*t1_it).piezas.end(); ++p1_it) {
            for (vector<Parte>::iterator pp1_it = (*p1_it).partes.begin(); pp1_it < (*p1_it).partes.end(); ++pp1_it) {
                int linea = 0;
                (*pp1_it).M = (*pp1_it).M0;
                vector<Vec3>::iterator l1_it = (*pp1_it).lineas.begin();
                for (vector<vector<vector<vector<bool>>>>::iterator sep4_it = (*pp1_it).PuedeSeparar.begin(); sep4_it < (*pp1_it).PuedeSeparar.end(); ++sep4_it, ++linea, ++l1_it) { // linea 1
                    int tipo2 = 0;
                    vector<TipoPieza>::iterator t2_it = tiposPieza.begin();
                    for (vector<vector<vector<bool>>>::iterator sep3_it = (*sep4_it).begin(); sep3_it < (*sep4_it).end(); ++sep3_it, ++tipo2, ++t2_it) { // tipo 2
                        int pieza2 = 0;
                        vector<PiezaG>::iterator p2_it = (*t2_it).piezas.begin();
                        for (vector<vector<bool>>::iterator sep2_it = (*sep3_it).begin(); sep2_it < (*sep3_it).end(); ++sep2_it, ++pieza2, ++p2_it) { // pieza 2
                            int parte2 = 0;
                            for (vector<bool>::iterator sep1_it = (*sep2_it).begin(); sep1_it < (*sep2_it).end(); ++sep1_it, ++parte2) { // parte 2
                                if (*sep1_it) {
                                    (*pp1_it).M[linea][tipo2][pieza2][parte2] += abs((*l1_it).ay - (*l1_it).by) * miLx;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

// Modelos completos

void DTM(Solucion& sol) {

    RecalcularM();

    // Envoltura

    IloEnv env;
    fileName += "_DTM";
    ofstream fout("Logs/" + fileName + ".log");

    try {
        // Modelo

        IloModel model(env);
        IloCplex cplex(model);

        // Variables

        IloNumVar Lx(env, minLx, maxLx, ILOFLOAT);
        IloNumVarArray3 x(env, tiposPieza.size());
        IloNumVarArray3 y(env, tiposPieza.size());
        IloBoolVarArray3 w(env, tiposPieza.size());
        int i = 0;
        for (vector<TipoPieza>::iterator t_it = tiposPieza.begin(); t_it < tiposPieza.end(); ++t_it, ++i) {
            int j = 0;
            x[i] = IloNumVarArray2(env, (*t_it).piezas.size());
            y[i] = IloNumVarArray2(env, (*t_it).piezas.size());
            w[i] = IloBoolVarArray2(env, (*t_it).piezas.size());
            for (vector<PiezaG>::iterator p_it = (*t_it).piezas.begin(); p_it < (*t_it).piezas.end(); ++p_it, ++j) {
                x[i][j] = IloNumVarArray(env, (*t_it).cantidad0, (*p_it).dmin.x, maxLx - (*p_it).dmax.x, ILOFLOAT);
                y[i][j] = IloNumVarArray(env, (*t_it).cantidad0, (*p_it).dmin.y, Ly - (*p_it).dmax.y, ILOFLOAT);
                w[i][j] = IloBoolVarArray(env, (*t_it).cantidad0);
            }
        }
        IloBoolVarArray9 u(env, tiposPieza.size());
        i = 0;
        for (vector<TipoPieza>::iterator t_it = tiposPieza.begin(); t_it < tiposPieza.end(); ++t_it, ++i) {
            int j = 0;
            u[i] = IloBoolVarArray8(env, (*t_it).piezas.size());
            for (vector<PiezaG>::iterator p_it = (*t_it).piezas.begin(); p_it < (*t_it).piezas.end(); ++p_it, ++j) {
                u[i][j] = IloBoolVarArray7(env, (*t_it).cantidad0);
                for (int k = 0; k < (*t_it).cantidad0; ++k) {
                    u[i][j][k] = IloBoolVarArray6(env, (*p_it).partes.size());
                    int l = 0;
                    for (vector<Parte>::iterator pp_it = (*p_it).partes.begin(); pp_it < (*p_it).partes.end(); ++pp_it, ++l) {
                        u[i][j][k][l] = IloBoolVarArray5(env, (*pp_it).lineas.size());
                        int m = 0;
                        for (vector<Vec3>::iterator l_it = (*pp_it).lineas.begin(); l_it < (*pp_it).lineas.end(); ++l_it, ++m) {
                            u[i][j][k][l][m] = IloBoolVarArray4(env, tiposPieza.size());
                            int ii = 0;
                            for (vector<TipoPieza>::iterator t2_it = tiposPieza.begin(); t2_it < tiposPieza.end(); ++t2_it, ++ii) {
                                u[i][j][k][l][m][ii] = IloBoolVarArray3(env, (*t2_it).piezas.size());
                                int jj = 0;
                                for (vector<PiezaG>::iterator p2_it = (*t2_it).piezas.begin(); p2_it < (*t2_it).piezas.end(); ++p2_it, ++jj) {
                                    u[i][j][k][l][m][ii][jj] = IloBoolVarArray2(env, (*t2_it).cantidad0);
                                    for (int kk = 0; kk < (*t2_it).cantidad0; ++kk) {
                                        u[i][j][k][l][m][ii][jj][kk] = IloBoolVarArray(env, (*p2_it).partes.size());
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // Funci�n objetivo

        model.add(IloMinimize(env, Lx));

        // Restricciones: Contenci�n en x

        i = 0;
        for (vector<TipoPieza>::iterator t_it = tiposPieza.begin(); t_it < tiposPieza.end(); ++t_it, ++i) {
            int j = 0;
            for (vector<PiezaG>::iterator p_it = (*t_it).piezas.begin(); p_it < (*t_it).piezas.end(); ++p_it, ++j) {
                for (int k = 0; k < (*t_it).cantidad0; ++k) {
                    model.add(x[i][j][k] + (*p_it).dmax.x <= Lx);
                }
            }
        }

        // Restricciones: Cantidad de piezas

        i = 0;
        for (vector<TipoPieza>::iterator t_it = tiposPieza.begin(); t_it < tiposPieza.end(); ++t_it, ++i) {
            for (int k = 0; k < (*t_it).cantidad0; ++k) {
                IloExpr sumw(env);
                for (int j = 0; j < (*t_it).piezas.size(); ++j) {
                    sumw += w[i][j][k];
                }
                model.add(sumw == 1); // Cantidad de piezas
                sumw.end();
            }
        }

        // Restricciones: No solapamiento

        i = 0;
        for (vector<TipoPieza>::iterator t_it = tiposPieza.begin(); t_it < tiposPieza.end(); ++t_it, ++i) {
            int j = 0;
            for (vector<PiezaG>::iterator p_it = (*t_it).piezas.begin(); p_it < (*t_it).piezas.end(); ++p_it, ++j) {
                for (int k = 0; k < (*t_it).cantidad0; ++k) {
                    int ii = i;
                    for (vector<TipoPieza>::iterator t2_it = t_it; t2_it < tiposPieza.end(); ++t2_it, ++ii) {
                        int jj = 0;
                        for (vector<PiezaG>::iterator p2_it = (*t2_it).piezas.begin(); p2_it < (*t2_it).piezas.end(); ++p2_it, ++jj) {
                            for (int kk = 0; kk < (*t2_it).cantidad0; ++kk) {
                                if (ii > i || (ii == i && jj > j && (*t_it).cantidad0 > 1) || (ii == i && jj == j && kk > k)) {
                                    int l = 0;
                                    for (vector<Parte>::iterator pp_it = (*p_it).partes.begin(); pp_it < (*p_it).partes.end(); ++pp_it, ++l) {
                                        int ll = 0;
                                        for (vector<Parte>::iterator pp2_it = (*p2_it).partes.begin(); pp2_it < (*p2_it).partes.end(); ++pp2_it, ++ll) {
                                            IloExpr sumu(env);
                                            int m = 0;
                                            for (vector<Vec3>::iterator l_it = (*pp_it).lineas.begin(); l_it < (*pp_it).lineas.end(); ++l_it, ++m) {
                                                if ((*pp_it).PuedeSeparar[m][ii][jj][ll]) {
                                                    model.add(((*l_it).ax - (*l_it).bx) * (y[i][j][k] - y[ii][jj][kk])
                                                        - ((*l_it).ay - (*l_it).by) * (x[i][j][k] - x[ii][jj][kk])
                                                        + (*pp_it).C2[m][ii][jj][ll] <= (*pp_it).M[m][ii][jj][ll] * (1 - u[i][j][k][l][m][ii][jj][kk][ll]));
                                                    sumu += u[i][j][k][l][m][ii][jj][kk][ll];
                                                }
                                            }
                                            m = 0;
                                            for (vector<Vec3>::iterator l_it = (*pp2_it).lineas.begin(); l_it < (*pp2_it).lineas.end(); ++l_it, ++m) {
                                                if ((*pp2_it).PuedeSeparar[m][i][j][l]) {
                                                    model.add(((*l_it).ax - (*l_it).bx) * (y[ii][jj][kk] - y[i][j][k])
                                                        - ((*l_it).ay - (*l_it).by) * (x[ii][jj][kk] - x[i][j][k])
                                                        + (*pp2_it).C2[m][i][j][l] <= (*pp2_it).M[m][i][j][l] * (1 - u[ii][jj][kk][ll][m][i][j][k][l]));
                                                    sumu += u[ii][jj][kk][ll][m][i][j][k][l];
                                                }
                                            }
                                            model.add(sumu <= w[i][j][k]);
                                            model.add(sumu <= w[ii][jj][kk]);
                                            model.add(sumu >= w[i][j][k] + w[ii][jj][kk] - 1);
                                            sumu.end();
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // Restricciones: Eliminaci�n de simetr�as

        i = 0;
        for (vector<TipoPieza>::iterator t_it = tiposPieza.begin(); t_it < tiposPieza.end(); ++t_it, ++i) {
            for (int j = 0; j < (*t_it).piezas.size(); ++j) {
                for (int k1 = 1; k1 < (*t_it).cantidad0; ++k1) {
                    for (int k2 = 0; k2 < k1; ++k2) {
                        model.add(x[i][j][k1] + maxLx * (1 - w[i][j][k1]) >= x[i][j][k2] - maxLx * (1 - w[i][j][k2])); // Simetría
                    }
                }
            }
            //for (int k = 1; k < (*t_it).cantidad0; ++k) {
            //    for (int j1 = 0; j1 < (*t_it).piezas.size(); ++j1) {
            //        for (int j2 = 0; j2 < (*t_it).piezas.size(); ++j2) {
            //            model.add(x[i][j1][k] + maxLx * (1 - w[i][j1][k]) >= x[i][j2][k - 1] - maxLx * (1 - w[i][j2][k - 1])); // Simetr�a
            //        }
            //    }
            //}
            
        }

        // Par�metros de ejecuci�n

        cplex.setParam(IloCplex::TiLim, maxTime);
        cplex.setParam(IloCplex::Threads, nThreads);
        cplex.setOut(fout);
        cplex.setWarning(fout);
        cplex.setError(fout);

        // Resolver el modelo

        cplex.solve();

        // Tiempo

        UpdateTime();

        // Manejo de resultados

        cout << cplex.getStatus() << endl;
        fout << cplex.getStatus() << endl;
        //cout << cplex.getStatus() << " Fun Obj = " << cplex.getObjValue() << endl;
        if (cplex.getStatus() == IloAlgorithm::Feasible || cplex.getStatus() == IloAlgorithm::Optimal) {

            // Guardar datos

            int id = 0;
            i = 0;
            for (vector<TipoPieza>::iterator t_it = tiposPieza.begin(); t_it < tiposPieza.end(); ++t_it, ++i) {
                int j = 0;
                for (vector<PiezaG>::iterator p_it = (*t_it).piezas.begin(); p_it < (*t_it).piezas.end(); ++p_it, ++j) {
                    for (int k = 0; k < (*t_it).cantidad0; ++k) {
                        if (cplex.getValue(w[i][j][k]) > 0.5) {
                            sol.piezasEmpacadas.push_back(PiezaEmpacada(&*p_it, Vec2(cplex.getValue(x[i][j][k]), cplex.getValue(y[i][j][k])), id));
                            ++id;
                        }
                    }
                }
            }
            sol.Lx = cplex.getObjValue();
            //sol.DeterminarLineasSeparadoras();

            // Files

            WriteSolution(sol, cplex.getMIPRelativeGap());
        }
        cplex.end();
    }
    catch (IloException& e) {
        fout << "Concert exception caught: " << e << endl;
    }
    catch (...) {
        fout << "Unknown exception caught" << endl;
    }
    fout.close();
    env.end();
}

// MAIN

void ResetearVariablesGlobales() {
    Ly = 0.0f;
    minLx = -1.0f;
    maxLx = 0.0f;
    //fileName = "";
    timeIni = std::chrono::system_clock::now();

    tiposPieza.clear();
    totalPiezas = 0;
    totalLineasSeparadoras = 0; // No es del modelo completo
    nVariables.clear();
    miny0 = 0;
    maxy0 = 0;
}
int main(int argc, char** argv) {
    timeIni = std::chrono::system_clock::now();

    double areaTotalPiezas = 0;

    // Par�metros

    for (int i = 1; i < argc - 1; i += 2) {
        if (argc - 1 >= i + 1) {
            if (string(argv[i]) == "-t") maxTime = atoi(argv[i + 1]);
            else if (string(argv[i]) == "-nThreads") nThreads = atoi(argv[i + 1]);
            else if (string(argv[i]) == "-ins") fileName = argv[i + 1];
            else if (string(argv[i]) == "-area") areaTotalPiezas = atof(argv[i + 1]);
            else {
                cout << "MAL EN PARAMETROS" << endl;
                return 0;
            }
        }
    }

    // Lectura de datos

    ResetearVariablesGlobales();
    ReadData();

    // Control de errores

    if (tiposPieza.size() == 0) {
        cout << "No se pudo leer el archivo" << endl;
        return 0;
    }
    if (areaTotalPiezas < errorG) {
        cout << "No se indic� el �rea total de las piezas" << endl;
        return 0;
    }

    // Determinar par�metros

    minLx = areaTotalPiezas / Ly;
    for (vector<TipoPieza>::iterator tp_it = tiposPieza.begin(); tp_it < tiposPieza.end(); ++tp_it) {
        minLx = max(minLx, (*tp_it).minx);
    }
    CalcularParametros();

    // Probar modelos completos

    cout << "Entra a DTM" << endl;
    timeIni = std::chrono::system_clock::now();
    Solucion cherri;
    DTM(cherri);
    cout << "Sale de DTM" << endl;
}
//END
