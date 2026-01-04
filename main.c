#include "raylib.h"
#include <math.h>
#include <stdio.h> 

//CONFIGURACIÓN DE LA INTERFAZ
#define SCREEN_W 1200 
#define SCREEN_H 700
#define HISTORY_LEN 550 
#define SIDEBAR_W 340 
#define TOPBAR_H 60


// Colores para la interfaz 
#define COLOR_BG        (Color){ 240, 242, 245, 255 } 
#define COLOR_SIDEBAR   (Color){ 255, 255, 255, 255 } 
#define COLOR_TOPBAR    (Color){ 33, 37, 41, 255 }    
#define COLOR_ACCENT    (Color){ 13, 110, 253, 255 }  
#define COLOR_MASS      (Color){ 220, 53, 69, 255 }   
#define COLOR_TEXT_MAIN (Color){ 33, 37, 41, 255 }    
#define COLOR_TEXT_SUB  (Color){ 108, 117, 125, 255 } 
#define COLOR_FORCE_K   (Color){ 25, 135, 84, 255 }   
#define COLOR_FORCE_C   (Color){ 255, 193, 7, 255 }

typedef struct {
    float y; // Posición (m)
    float v; // Velocidad (m/s)
} Estado;  // Estado dinamico de posicion y velocidad



typedef struct {
    float m; // Masa (kg)
    float k; // Rigidez (N/m)
    float c; // Amortiguamiento (N*s/m)
} Parametros;  // Propiedades fisicas constantes

// SEGUNDA LEY DE NEWTON
// Basandise en la ecuacion : m*a + c*v + k*y = 0
// Y despejando la aceleracion (a) : a=(-k*y - c*v)/m
float CalcularAceleracion(Estado st, Parametros p) {
    float fuerza_resorte = -p.k * st.y;  // Ley de hooke
    float fuerza_amortiguador = -p.c * st.v; // Fuerza de oposición proporcional a la velocidad
    // a = F_total / m
    return (fuerza_resorte + fuerza_amortiguador) / p.m;
}

// Runge-Kutta de 4to Orden

Estado PasoRK4(Estado actual, Parametros p, float dt) {
    // Paso 1: Pendiente al inicio del intervalo
    float v1 = actual.v;
    float a1 = CalcularAceleracion(actual, p);

    // Paso 2: Pendiente en el munto medio usando v1 y a1
    Estado s2 = { actual.y + v1 * 0.5f * dt, actual.v + a1 * 0.5f * dt };
    float v2 = s2.v;
    float a2 = CalcularAceleracion(s2, p);

    // Paso 3: Otra oendiente en el punto medio usando v2 y a2
    Estado s3 = { actual.y + v2 * 0.5f * dt, actual.v + a2 * 0.5f * dt };
    float v3 = s3.v;
    float a3 = CalcularAceleracion(s3, p);
    
    // Paso 4: Pediente al final del intervalo usando v3 y a3
    Estado s4 = { actual.y + v3 * dt, actual.v + a3 * dt };
    float v4 = s4.v;
    float a4 = CalcularAceleracion(s4, p);

    //Promedio de las 4 pendientes para el nuevvo estado (ponderado)
    Estado nuevo;
    nuevo.y = actual.y + (dt / 6.0f) * (v1 + 2*v2 + 2*v3 + v4);
    nuevo.v = actual.v + (dt / 6.0f) * (a1 + 2*a2 + 2*a3 + a4);
    return nuevo;
}


// PARTE VISUAL
// Dibuja el resorte
void DibujarResorteMecanico(Vector2 inicio, Vector2 fin, int numEspiras) {
    float longitud = fin.y - inicio.y;
    float espaciado = longitud / (numEspiras + 1);
    float anchoEspira = 22.0f;
    float grosorAnillo = 5.0f;
    
    for (int i = 1; i <= numEspiras; i++) {
        float yCenter = inicio.y + (i * espaciado);
        Rectangle rectAnillo = {inicio.x - anchoEspira/2, yCenter - grosorAnillo/2, anchoEspira, grosorAnillo};
        DrawRectangleRounded(rectAnillo, 0.5f, 4, (Color){90, 90, 90, 255}); 
        DrawLine(rectAnillo.x + 2, rectAnillo.y + 1, rectAnillo.x + rectAnillo.width - 2, rectAnillo.y + 1, (Color){150, 150, 150, 255});
    }
}


// Función UI
void DibujarDatoUI(const char* label, float val, const char* unit, const char* subtitle, const char* keys, int idx) {
    int startY = TOPBAR_H + 40;
    int spacing = 95;
    int y = startY + (idx * spacing);
    char buffer[50];
    DrawText(label, 30, y, 18, COLOR_TEXT_MAIN);
    DrawText(keys, SIDEBAR_W - 100, y+2, 12, COLOR_ACCENT);
    sprintf(buffer, "%.1f %s", val, unit);
    DrawText(buffer, 30, y + 25, 28, COLOR_TEXT_MAIN);
    DrawText(subtitle, 30, y + 58, 14, COLOR_TEXT_SUB);
    if (idx < 2) DrawRectangle(30, y + 85, SIDEBAR_W - 60, 1, (Color){230,230,230,255});
}

// MAIN
int main(void) {
    InitWindow(SCREEN_W, SCREEN_H, "Simulación de un sistema RMA");
    SetTargetFPS(60);

    Parametros params = { 5.0f, 15.0f, 1.0f }; 
    Estado estado = { 50.0f, 0.0f };

    float historial[HISTORY_LEN] = { 0 };
    
    // CORRECCIÓN DE POSICIÓN
    float espacioLibre = SCREEN_W - SIDEBAR_W;
    Vector2 anclaje = { SIDEBAR_W + (espacioLibre / 4.0f), 100 }; 
    int framesGolpe = 0; 

    while (!WindowShouldClose()) {
        //1. INPUT
        if (IsKeyPressed(KEY_R)) {
            estado.y = 100.0f; estado.v = 0.0f;
            for(int i=0; i<HISTORY_LEN; i++) historial[i] = 0;
        }
        if (IsKeyPressed(KEY_SPACE)) { estado.v += 250.0f; framesGolpe = 15; }

        if (IsKeyDown(KEY_UP)) params.k += 0.5f;
        if (IsKeyDown(KEY_DOWN)) params.k -= 0.5f;
        if (IsKeyDown(KEY_RIGHT)) params.c += 0.1f;
        if (IsKeyDown(KEY_LEFT)) params.c -= 0.1f;
        if (IsKeyDown(KEY_W)) params.m += 0.1f;
        if (IsKeyDown(KEY_S)) params.m -= 0.1f;

        if (params.k < 1.0f) params.k = 1.0f;
        if (params.c < 0.0f) params.c = 0.0f;
        if (params.m < 0.5f) params.m = 0.5f;

        //2. FÍSICA
        for(int i=0; i<4; i++) estado = PasoRK4(estado, params, 0.1f);
        for (int i = 0; i < HISTORY_LEN - 1; i++) historial[i] = historial[i+1];
        historial[HISTORY_LEN - 1] = estado.y;

        //3. DIBUJADO
        BeginDrawing();
        ClearBackground(COLOR_BG);

        // Barra Superior
        DrawRectangle(0, 0, SCREEN_W, TOPBAR_H, COLOR_TOPBAR);
        DrawText("Sistema Masa-Resorte-Amortiguador", 20, 15, 20, WHITE);
        DrawText("Modelo: my'' + cy' + ky = 0", SCREEN_W - 250, 20, 16, LIGHTGRAY);

        // Sidebar
        DrawRectangle(0, TOPBAR_H, SIDEBAR_W, SCREEN_H - TOPBAR_H, COLOR_SIDEBAR);
        DrawRectangleGradientH(SIDEBAR_W, TOPBAR_H, 10, SCREEN_H - TOPBAR_H, Fade(BLACK, 0.1f), Fade(BLACK, 0.0f));

        DibujarDatoUI("Masa (Inercia)", params.m, "kg", "Resistencia al cambio de velocidad.\nDetermina el periodo natural.", "[W / S]", 0);
        DibujarDatoUI("Rigidez (k)", params.k, "N/m", "Capacidad de almacenar energía\npotencial. Fuerza restauradora.", "[ARR / ABJ]", 1);
        DibujarDatoUI("Amortiguamiento (c)", params.c, "N*s/m", "Controla el decaimiento.", "[IZQ / DER]", 2);

        // Diagnóstico rápido (Factor Zeta)
        float critico = 2 * sqrtf(params.m * params.k);
        float zeta = params.c / critico;
        DrawRectangleLines(20, SCREEN_H - 150, SIDEBAR_W - 40, 110, Fade(COLOR_TEXT_SUB, 0.3f));
        DrawText("Estado del Sistema:", 35, SCREEN_H - 135, 16, COLOR_TEXT_MAIN);
        const char* tipo = (zeta == 0) ? "SIN AMORTIGUAMIENTO" : (zeta < 1.0f) ? "SUB-AMORTIGUADO" : "SOBRE-AMORTIGUADO";
        DrawText(tipo, 35, SCREEN_H - 75, 16, (zeta < 1.0f) ? ORANGE : GREEN);

        // Simulación Física
        float cajaY = anclaje.y + 180 + estado.y;
        float boxSize = 60 + (params.m * 2);

        // Soporte fijo
        DrawRectangle(anclaje.x - 100, (int)anclaje.y - 15, 200, 15, (Color){60,60,60,255});
        DrawText("Soporte Fijo", anclaje.x - 40, (int)anclaje.y - 35, 14, COLOR_TEXT_SUB);

        // Componentes Mecánicos (Resorte y Amortiguador)
        DibujarResorteMecanico(anclaje, (Vector2){anclaje.x - 30, cajaY}, 12);
        DrawLineEx((Vector2){anclaje.x + 30, anclaje.y}, (Vector2){anclaje.x + 30, cajaY}, 3.0f, Fade(GRAY, 0.7f)); 
        DrawRectangle(anclaje.x + 18, (int)anclaje.y + 50, 24, 60, GRAY); 

        // Masa
        Rectangle recMasa = { anclaje.x - boxSize/2, cajaY, boxSize, boxSize };
        DrawRectangleRounded((Rectangle){recMasa.x+4, recMasa.y+4, recMasa.width, recMasa.height}, 0.2f, 8, Fade(BLACK, 0.2f)); // Sombra
        DrawRectangleRounded(recMasa, 0.2f, 8, COLOR_MASS);
        DrawText("m", anclaje.x - 5, cajaY + boxSize/2 - 10, 20, WHITE);

        if (framesGolpe > 0) { DrawText("¡IMPULSO!", anclaje.x - 40, cajaY + boxSize + 10, 16, COLOR_ACCENT); framesGolpe--; }

        // Gráfica
        int graphX = anclaje.x + 220;
        int graphY = SCREEN_H / 2 + 50;
        int graphW = SCREEN_W - graphX - 40;
        int graphH = 250;
        DrawRectangle(graphX, graphY - graphH/2, graphW, graphH, WHITE);
        DrawRectangleLines(graphX, graphY - graphH/2, graphW, graphH, Fade(LIGHTGRAY, 0.5f));
        DrawLine(graphX, graphY, graphX + graphW, graphY, (Color){200, 200, 200, 255}); // Línea y=0
        
        for (int i = 0; i < HISTORY_LEN - 1; i++) {
            Vector2 p1 = { (float)graphX + ((float)i/HISTORY_LEN)*graphW, (float)graphY + historial[i]*0.7f };
            Vector2 p2 = { (float)graphX + ((float)(i+1)/HISTORY_LEN)*graphW, (float)graphY + historial[i+1]*0.7f };
            DrawLineEx(p1, p2, 2.0f, COLOR_ACCENT);
        }
        DrawText("Historial de Posición", graphX, graphY - graphH/2 - 25, 18, COLOR_TEXT_MAIN);

        // Pie de página
        DrawRectangle(0, SCREEN_H - 30, SCREEN_W, 30, (Color){230,230,230,255});
        DrawText("CONTROLES: [ESPACIO] Impulso | [R] Reiniciar | Ajuste parámetros con teclado.", 20, SCREEN_H - 20, 12, COLOR_TEXT_SUB);

        EndDrawing();
    }
    CloseWindow();
    return 0;
}