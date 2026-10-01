# Exploración de cfg_0

Ejecutar desde rds2026:

    python3 -m P01.part1

También admite ejecución directa desde cualquier carpeta:

    python3 /home/alfredo/Desktop/4º/servicios/p01/rds2026/P01/part1.py

Q cierra, ESPACIO pausa/reanuda y D activa la depuración. La exploración termina automáticamente. Se guardan resultado.json y resultado.png en P01. Para repetir rápidamente la prueba con el simulador real, sin limitar FPS:

    python3 -m P01.part1 --headless
    python3 P01/verificar_cobertura.py

## Comportamiento

Usa únicamente odometría y proximidad para planificar. Guarda un grafo de posiciones separadas 0,5 unidades, toma lecturas frontales y laterales y comprueba también la parte trasera cuando es desconocida. Busca la frontera más cercana mediante BFS sobre trayectos conocidos. A igual distancia favorece continuar recto. Refresca sensores después de girar. La velocidad sigue siendo 0,5 unidades por actualización; no teletransporta ni atraviesa obstáculos. Los giros son instantáneos, como permite rotate() en el simulador.

En el ejercicio se crea una copia independiente del diccionario de sensores y se inicializa su odometría a (21,21), porque la clase original comienza en (0,0). No se modifica vacuum, ninguna clase del simulador, cfg_0, Comportamientos.py ni followWall.py.

## Resultados

| Variante | Pasos/actualizaciones | Celdas cubiertas | Colisiones | Finaliza |
|---|---:|---:|---:|---|
| Comportamiento anterior (con la misma odometría inicial corregida) | 5000 | 116 | 1 | No, límite de prueba |
| Frontera próxima y preferencia por avanzar recto | 1421 | 416 | 0 | Sí |

La ejecución visible a 32 FPS tardó 44,35 segundos. El tiempo nominal es 44,41 segundos. Una repetición acelerada produjo exactamente los mismos 1421 pasos, 416 celdas y cero colisiones. Son 1247 posiciones del centro visitadas y 186 llamadas de giro, incluidas las lecturas traseras.

Se probó también priorizar nuevas celdas a igual distancia: obtuvo 416 celdas a los 1434 pasos, pero encontró una discrepancia de transitabilidad al final. Se descartó y no está disponible en la versión entregada.

El verificador independiente utiliza geometría de obstáculos y tamaño 2x2 para calcular la componente accesible sobre una rejilla de 0,5 y comparar la unión de celdas con la salida. Confirma las mismas 416 celdas, sin faltantes ni adicionales. El controlador nunca consulta ese mapa. Las posiciones exactas del centro no coinciden todas con el modelo geométrico ideal por el redondeo de rectángulos de Pygame y los errores de coma flotante del simulador: esta comprobación certifica cobertura de celdas, no igualdad de todos los centros posibles.

El mapa JSON distingue posiciones transitables y posiciones del centro bloqueadas (coordenadas multiplicadas por dos). Una posición bloqueada no significa necesariamente una pared en ese punto: significa que el cuerpo del robot no cabe. covered_tiles contiene las celdas del sensor en coordenadas normales. El PNG marca de verde las celdas cubiertas.

Es la mejor variante funcional de las probadas para cfg_0 y el inicio (21,21), no una demostración del mínimo global. La prueba rápida usa las mismas actualizaciones y colisiones; solamente elimina la espera entre fotogramas. El comportamiento original se reproduce con --strategy original --max-steps 5000.
