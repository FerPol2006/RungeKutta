#include "raylib.h"
#include <math.h>
#include <stdio.h> 

// --- CONFIGURACIÓN ---
#define SCREEN_W 1000
#define SCREEN_H 600
#define HISTORY_LEN 500 

typedef struct {
    float y; // Posición (m)
    float v; // Velocidad (m/s)
} Estado;

typedef struct {
    float m; // Masa (kg)
    float k; // Rigidez (N/m)
    float c; // Amortiguamiento (N*s/m)
} Parametros;

// --- FÍSICA (RK4) ---
float CalcularAceleracion(Estado st, Parametros p) {
    float fuerza_resorte = -p.k * st.y;
    float fuerza_amortiguador = -p.c * st.v;
    // a = F_total / m
    return (fuerza_resorte + fuerza_amortiguador) / p.m;
}

Estado PasoRK4(Estado actual, Parametros p, float dt) {
    float v1 = actual.v;
    float a1 = CalcularAceleracion(actual, p);

    Estado s2 = { actual.y + v1 * 0.5f * dt, actual.v + a1 * 0.5f * dt };
    float v2 = s2.v;
    float a2 = CalcularAceleracion(s2, p);

    Estado s3 = { actual.y + v2 * 0.5f * dt, actual.v + a2 * 0.5f * dt };
    float v3 = s3.v;
    float a3 = CalcularAceleracion(s3, p);

    Estado s4 = { actual.y + v3 * dt, actual.v + a3 * dt };
    float v4 = s4.v;
    float a4 = CalcularAceleracion(s4, p);

    Estado nuevo;
    nuevo.y = actual.y + (dt / 6.0f) * (v1 + 2*v2 + 2*v3 + v4);
    nuevo.v = actual.v + (dt / 6.0f) * (a1 + 2*a2 + 2*a3 + a4);
    return nuevo;
}

// --- DIBUJO ---
void DibujarResorte(Vector2 inicio, Vector2 fin, int segmentos, float ancho, Color color) {
    Vector2 actual = inicio;
    float distY = (fin.y - inicio.y) / segmentos;
    for (int i = 0; i < segmentos; i++) {
        float dir = (i % 2 == 0) ? 1.0f : -1.0f;
        Vector2 siguiente = { inicio.x + (dir * ancho), actual.y + distY };
        if (i == segmentos - 1) siguiente.x = fin.x;
        DrawLineEx(actual, siguiente, 2.0f, color);
        actual = siguiente;
    }
}

// --- MAIN ---
int main(void) {
    InitWindow(SCREEN_W, SCREEN_H, "Lab. Vibraciones Mecánicas - Controles Completos");
    SetTargetFPS(60);

    Parametros params = { 5.0f, 15.0f, 1.0f }; // m=5kg inicial
    Estado estado = { 100.0f, 0.0f };

    float historial[HISTORY_LEN] = { 0 };
    Vector2 anclaje = { SCREEN_W * 0.25f, 100 }; 
    int framesGolpe = 0; 

    while (!WindowShouldClose()) {
        // --- 1. INPUT ---
        if (IsKeyPressed(KEY_R)) {
            estado.y = 150.0f; estado.v = 0.0f;
            for(int i=0; i<HISTORY_LEN; i++) historial[i] = 0;
        }

        if (IsKeyPressed(KEY_SPACE)) {
            estado.v += 200.0f; // Impulso de velocidad
            framesGolpe = 20; 
        }

        // Rigidez (K) - Flechas Arriba/Abajo
        if (IsKeyDown(KEY_UP)) params.k += 0.5f;
        if (IsKeyDown(KEY_DOWN)) params.k -= 0.5f;
        if (params.k < 1.0f) params.k = 1.0f;

        // Amortiguamiento (C) - Flechas Izq/Der
        if (IsKeyDown(KEY_RIGHT)) params.c += 0.1f;
        if (IsKeyDown(KEY_LEFT)) params.c -= 0.1f;
        if (params.c < 0.0f) params.c = 0.0f;

        // --- NUEVO: Masa (M) - Teclas W/S ---
        if (IsKeyDown(KEY_W)) params.m += 0.1f;
        if (IsKeyDown(KEY_S)) params.m -= 0.1f;
        if (params.m < 0.5f) params.m = 0.5f; // Mínimo 0.5kg

        // --- 2. FÍSICA ---
        for(int i=0; i<4; i++) estado = PasoRK4(estado, params, 0.1f);

        for (int i = 0; i < HISTORY_LEN - 1; i++) historial[i] = historial[i+1];
        historial[HISTORY_LEN - 1] = estado.y;

        // --- 3. DIBUJADO ---
        BeginDrawing();
        ClearBackground(RAYWHITE);

        float cajaY = anclaje.y + 200 + estado.y;
        
        // Estructura Soporte
        DrawLine(anclaje.x - 60, anclaje.y, anclaje.x + 60, anclaje.y, BLACK);
        
        // Componentes Mecánicos
        DibujarResorte(anclaje, (Vector2){anclaje.x, cajaY}, 12, 20, BLACK);
        
        // Amortiguador
        DrawLine(anclaje.x + 40, anclaje.y, anclaje.x + 40, cajaY, Fade(GRAY, 0.6f));
        DrawRectangle(anclaje.x + 35, anclaje.y, 10, 70, GRAY);
        DrawRectangle(anclaje.x + 35, cajaY - 70, 10, 70, GRAY);

        // MASA (La caja cambia de tamaño según el peso)
        // Base de 60px + 4px por cada kg extra
        float sizeBox = 60 + (params.m * 4); 
        DrawRectangle(anclaje.x - (sizeBox/2), cajaY, sizeBox, 50, MAROON);
        DrawText("M", anclaje.x - 5, cajaY + 15, 20, WHITE);

        // Efecto visual golpe
        if (framesGolpe > 0) {
            DrawText("IMPULSO", anclaje.x - 40, cajaY + 60, 20, RED);
            framesGolpe--;
        }

        // Gráfica
        int graphX = SCREEN_W / 2;
        int graphY = SCREEN_H / 2;
        DrawLine(graphX, graphY, SCREEN_W - 20, graphY, BLACK); 
        DrawLine(graphX, 100, graphX, SCREEN_H - 100, BLACK);   
        DrawText("Tiempo (t) ->", SCREEN_W - 100, graphY + 10, 10, BLACK);

        for (int i = 0; i < HISTORY_LEN - 1; i++) {
            DrawLine(graphX + i, graphY + (int)historial[i], 
                     graphX + i + 1, graphY + (int)historial[i+1], BLUE);
        }

        // --- UI / PANEL DE DATOS CON UNIDADES ---
        char buffer[100];
        DrawRectangle(10, 10, 360, 190, Fade(LIGHTGRAY, 0.5f));
        DrawRectangleLines(10, 10, 360, 190, GRAY);
        
        // 1. Masa
        sprintf(buffer, "Masa (m): %.1f kg [W/S]", params.m);
        DrawText(buffer, 20, 20, 20, DARKBLUE);

        // 2. Rigidez
        sprintf(buffer, "Rigidez (k): %.1f N/m [Arr/Abj]", params.k);
        DrawText(buffer, 20, 50, 20, DARKGRAY);
        
        // 3. Amortiguamiento
        sprintf(buffer, "Amortiguamiento (c): %.1f N*s/m [Izq/Der]", params.c);
        DrawText(buffer, 20, 80, 20, DARKGRAY);

        // 4. Posición Real
        // Dividimos por 100 para simular que 100px = 1 metro visualmente
        sprintf(buffer, "Posicion (y): %.2f m", estado.y / 100.0f); 
        DrawText(buffer, 20, 110, 20, MAROON);

        DrawText("ESPACIO: Aplicar fuerza externa", 20, 140, 10, DARKGRAY);
        DrawText("R: Reiniciar sistema", 20, 160, 10, DARKGRAY);

        // Diagnóstico Automático
        float critico = 2 * sqrt(params.m * params.k);
        int textoY = 210;
        
        // Usamos notación matemática
        if (params.c == 0) DrawText("SISTEMA NO AMORTIGUADO (c = 0)", 20, textoY, 20, RED);
        else if (params.c < critico) {
            sprintf(buffer, "SUB-AMORTIGUADO (zeta < 1)");
            DrawText(buffer, 20, textoY, 20, ORANGE);
        }
        else if (params.c > critico) {
            sprintf(buffer, "SOBRE-AMORTIGUADO (zeta > 1)");
            DrawText(buffer, 20, textoY, 20, BLUE);
        }
        else DrawText("AMORTIGUAMIENTO CRITICO (zeta = 1)", 20, textoY, 20, GREEN);

        EndDrawing();
    }
    CloseWindow();
    return 0;
}