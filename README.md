# Seguidor de Línea con PID Adaptativo

Proyecto de un seguidor de línea basado en control PID, con lectura de sensores IR a través de un multiplexor.  
El código está hecho en C++ para Arduino y ajusta sus parámetros de forma dinámica según la posición de la línea.

---

## Características

- Control PID adaptativo según el tipo de curva o pérdida de línea.  
- Lectura de sensores IR mediante un multiplexor (menos pines usados).  
- Parada automática al detectar negro durante más de 0.3 segundos.  
- Corrección suave de velocidad entre motores.  
- Priorización de sensores centrales para mantener trayectorias rectas.  

---

## Cómo funciona

1. Los sensores IR leen la línea y envían valores (0 o 1) al microcontrolador.  
2. Se calcula un error promedio ponderado según la posición de los sensores activos.  
3. El controlador PID genera una señal de corrección que ajusta la velocidad de cada motor.  
4. Si los sensores detectan solo negro por más de 0.3 s, el robot se detiene.  

---

## Conexiones principales

| Componente | Pines |
|-------------|--------|
| Motores A/B | 18, 19, 20, 21 |
| Encoders | 2, 3, 22, 23 |
| Multiplexor | S0: 4, S1: 5, S2: 0, Y: 1 |
| Botón inicio/parada | 14 |

---

## PID (valores base)

| Parámetro | Valor |
|------------|--------|
| Kp | 9.9 |
| Ki | 0.00000115 |
| Kd | 1.5 |

Estos valores funcionan bien como punto de partida, pero se pueden ajustar según la pista o la respuesta de los motores.

---

## Uso

1. Cargar el código en el Arduino.  
2. Colocar el robot sobre la pista.  
3. Presionar el botón de inicio.  
4. El robot seguirá la línea automáticamente.  
5. Si detecta una zona negra por más de 0.3 s, se detiene.

---

## Requisitos

- Arduino (UNO, MEGA o similar)  
- 8 sensores IR  
- Multiplexor (CD4051 o similar)  
- Driver de motores (L298N o L293D)  
- 2 motores DC con encoders  
- Fuente de alimentación estable

---
