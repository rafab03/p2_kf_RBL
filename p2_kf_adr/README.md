En este README se explicará cada uno de los archivos que se han modificado para modificar la práctica:

    --motion_models.py: Define los modelos dinámicos para ambos filtros:
        
        -velocity_motion_model(). Para el filtro 3D
            state_transition_matrix_A(): Matriz identidad 3x3
            control_input_matrix_B(mu, dt)

        -velocity_motion_model_2(). Para el filtro 6D
            A(dt): Matriz 6x6 que avanza posición y orientación según velocidades lineales y angulares.
            B(mu, dt)

    --observation_models.py: Contiene los modelos de observación (C) para el paso de corrección:
        
        -odometry_observation_model(): retorna una matriz identidad 3x3.

        -odometry_observation_model_2(): retorna una matriz identidad 6x6.

    --kf_estimation.py: Implementa un Filtro de Kalman clásico en 2D (x,y,θ). Flujo principal:

        Inicializa estado μ y covarianza Σ.

        Lee noise_config para fijar R (ruido de proceso) y Q (ruido de medición).

        Se suscribe a /odom, extrae (v,ω) y la pose 2D real.

        En cada mensaje:

            Calcula dtdt.

            predict(u, dt) y update(z) del filtro.

            Publica la pose filtrada en /kf_estimate como PoseWithCovarianceStamped.

            Llama a la clase Visualizer para dibujar pose real vs. estimada.

    --kf_estimation_vel.py: Usa un Filtro de Kalman ampliado a 6D (x,y,vx,vy,θ,ω).

        Diferencias frente a kf_estimation.py:

            Estado de 6 dimensiones, inicializado con ceros y covarianza 6×6.

            En la observación incluye también las velocidades (vx,vy,ω), generadas  por generate_noisy_measurement_2.

            La matriz A(dt) ya incorpora el modelo de movimiento uniforme (posición += velocidad·dt).

            Publica igualmente solo la pose filtrada (no las velocidades) en /kf_estimate, y actualiza el Visualizer.


    --kalman_filter.py: Este archivo unifica los pasos clásicos de predict–update para estados de 3 y 6 dimensiones, parametrizados por modelos sencillos de movimiento y observación lineales.

        -KalmanFilter (3D):

            Estado interno de tres valores: posición x, posición y y orientación θ.

            predict(u, dt): avanza el estado usando un modelo de velocidad simple y aumenta la incertidumbre con la matriz de ruido de proceso.

            update(z): corrige la predicción comparando con la medida (x, y, θ), calcula la ganancia de Kalman y ajusta tanto el estado como la covarianza con el ruido de medida.

        -KalmanFilter_2 (6D):

            Estado interno de seis valores: posición x, y, velocidades vx, vy, orientación θ y velocidad angular ω.

            predict(dt): integra posiciones con velocidades (x += vx⋅dt, y += vy⋅dt, θ += ω⋅dt), y ajusta la covarianza de proceso.

            update(z): corrige las seis componentes del estado usando mediciones directas de posición y velocidad, ajustando el estado y la covarianza con el ruido de medida.

A continuación se explicarán los resultados observados y se presentará unbreve análisis de por qué ocurre lo observado:

-Ruido de proceso: Modela la incertidumbre de la dinámica interna (cuánto de fiable es que la ecuación de movimiento describa realmente el sistema).
Un ruido de proceso grande inyecta mucha incertidumbre en la covarianza, de modo que el filtro “desconfía” de su propia predicción y deja espacio para correcciones posteriores. 

-Ruido de medida: Modela la incertidumbre de los sensores (cuánto de fiable es la observación). Un ruido de medida grande hace que el filtroconfíe menos en la medida y más en la predicción. Con un ruido de medida pequeño se logra que el filtro corrija con mucha fuerza cada observación.

Por tanto, tal como se aprecia en las capturas, con un ruido de medida grande, se hacen correcciones débiles. Los ruidos del GPS apenas alteran la predicción y se acaba con una línea muy ajustada a la real. Con un ruido de proceso alto, la trayectoria corregirá poco y diferirá más de la real.

Para el FK2, un ruido de proceso alto en la parte de velocidad indica al filtro que desconfíe de su propio modelo de velocidad. El filtro “frena” sus propias predicciones de velocidad, dejándose guiar más por la medición. Con un ruido de medida alto, el filtro asume mucha confianza en el modelo de velocidad, predice de forma “agresiva” el avance de la posición siguiendo la última velocidad estimada, y entonces las correcciones de medida tienen menos peso.

Si ambos ruidos tienen valores bajos (tanto el de proceso como el de medida), el filtro acaba confiando mucho tanto en el modelo como en los sensores. El resultado es un estimado muy reactivo: sigue casi al instante cualquier cambio, pero apenas atenúa el ruido.